"""送り手の存在と実際の指令を区別する。起動後に増えたDDS送り手は除外しない。"""
import time

TOPIC = "/api/sport/request"
BARE_DDS = "_CREATED_BY_BARE_DDS_APP_"
QUIET_SECONDS = 1.0


def endpoint_gid(endpoint):
    return bytes(endpoint.endpoint_gid)


class PublisherGuard:
    def __init__(self, own=0, clock=time.monotonic, topic=TOPIC,
                 read_only_apis=(), activity_only=False):
        self.own = own
        self.read_only_apis = frozenset(read_only_apis)
        self.activity_only = activity_only
        self.last_foreign_api = None
        self.topic = topic
        self.clock = clock
        self.native = set()
        self.last_seen = {}
        self.own_requests = {}
        self.unattributed = None
        self.frozen = False

    @staticmethod
    def request_key(message):
        return (int(message.header.identity.id), int(message.header.identity.api_id), message.parameter)

    def remember_own(self, message):
        now = self.clock()
        self.own_requests = {key: stamp for key, stamp in self.own_requests.items() if now - stamp < 5}
        self.own_requests[self.request_key(message)] = now

    def unknown_active(self):
        return self.unattributed is not None and self.clock() - self.unattributed < QUIET_SECONDS

    def on_request(self, message, info=None):
        # 古いHumbleはコールバックへメッセージだけを渡す。自分の送信はID/API/内容で識別する。
        if message is not None:
            api = int(message.header.identity.api_id)
            if api in self.read_only_apis:
                return
            sent = self.own_requests.get(self.request_key(message))
            if sent is not None and self.clock() - sent < 5:
                return
            self.last_foreign_api = api
        if info is not None and hasattr(info, "publisher_gid"):
            self.last_seen[bytes(info.publisher_gid)] = self.clock()
        else:
            self.unattributed = self.clock()

    def active_native(self):
        now = self.clock()
        if self.unknown_active():
            return set(self.native)
        return {source for source in self.native
                if source in self.last_seen and now - self.last_seen[source] < QUIET_SECONDS}

    def freeze(self, node):
        if self.frozen:
            return
        if hasattr(node, "get_publishers_info_by_topic"):
            self.native = {endpoint_gid(e) for e in node.get_publishers_info_by_topic(self.topic)
                           if e.node_name == BARE_DDS}
        self.frozen = True

    def conflicts(self, node):
        if self.activity_only:
            # 状態照会用にも送り手が作られるため、存在数では操作の競合を判断しない。
            now = self.clock()
            return int(self.unknown_active() or any(now - stamp < QUIET_SECONDS
                                                   for stamp in self.last_seen.values()))
        if not hasattr(node, "get_publishers_info_by_topic"):
            return max(0, node.count_publishers(self.topic) - self.own)
        endpoints = node.get_publishers_info_by_topic(self.topic)
        # 最初に存在したnative DDSでも、最近指令を送っていれば競合として扱う。
        active = self.active_native()
        idle = {endpoint_gid(e) for e in endpoints if e.node_name == BARE_DDS
                and endpoint_gid(e) in self.native and endpoint_gid(e) not in active}
        return max(int(self.unknown_active()), len(endpoints) - len(idle) - self.own, 0)
