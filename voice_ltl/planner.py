"""Pick the next move on a supervisor: shortest controllable path to a marked state."""

from collections import deque

from voice_ltl.automata import Automaton


class PlanError(Exception):
    """The event is not allowed in the supervisor's current state."""


class Runtime:
    """Follows a supervisor (state 0 is initial) as events happen."""

    def __init__(self, sup: Automaton):
        self.sup = sup
        self.state = 0
        self._out: dict[int, dict[str, int]] = {}
        for src, event, dst in sup.trans:
            self._out.setdefault(src, {})[event] = dst

    @property
    def done(self) -> bool:
        return self.state in self.sup.marked

    def allowed(self) -> list[str]:
        return sorted(self._out.get(self.state, {}))

    def plan(self, avoid: frozenset[tuple[int, str]] = frozenset()) -> list[str] | None:
        """Shortest controllable event list to a marked state (BFS), [] if already there.

        Ties go to the event whose name sorts last at the first step where paths differ
        (the lecture's expected route goes up before down). Returns None when no path exists.
        `avoid` holds (state, event) pairs whose transitions must not be used.
        """
        if self.done:
            return []
        prev: dict[int, tuple[int, str]] = {}
        seen = {self.state}
        queue = deque([self.state])
        while queue:
            src = queue.popleft()
            for event in sorted(self._out.get(src, {}), reverse=True):
                if event in self.sup.uncontrollable or (src, event) in avoid:
                    continue
                dst = self._out[src][event]
                if dst in seen:
                    continue
                seen.add(dst)
                prev[dst] = (src, event)
                if dst in self.sup.marked:
                    path = []
                    node = dst
                    while node != self.state:
                        node, ev = prev[node]
                        path.append(ev)
                    return path[::-1]
                queue.append(dst)
        return None

    def next_event(self) -> str | None:
        path = self.plan()
        return path[0] if path else None

    def apply(self, event: str) -> None:
        nxt = self._out.get(self.state, {}).get(event)
        if nxt is None:
            raise PlanError(f"{event} is not allowed in state {self.state}")
        self.state = nxt
