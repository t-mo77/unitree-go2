import math
import re
from pathlib import Path

import numpy as np
import pytest

from go2_nl_control.actions import (BALANCE_STAND, HELLO, LYING, MOVE, RISE_SIT, SIT, SITTING,
                                    STAND_DOWN, STAND_UP, STOP_MOVE, state_from_error_code)
from go2_nl_control.perception import PerceptionConfig
from gazebo.bridge_core import (DOWN_CMD, FALL_TILT, FALL_Z, GROUND_Z, LIDAR_IN_BODY, RELEASE_ACC,
                                STAND_CMD, TROT_CMD, TROT_REPORT_LATE_S, TROT_WAIT_S, UPRIGHT_Z,
                                V_X_MAX, V_Y_MAX, W_Z_MAX, BoxWorld, BridgeCore,
                                drop_ground_and_self_points, drop_self_points, move_input)
from sim.sim_core import FOOTPRINT
from voice_ltl.grid_executor import GRID_EXEC_CONFIG

XACRO = Path(__file__).resolve().parent.parent / "gazebo" / "go2_sensors.urdf.xacro"


def standing_core() -> BridgeCore:
    """歩くに入って（3.6）TROT_WAIT_S より後（4.2）まで進めた BridgeCore。"""
    b = BridgeCore(0.0)
    b.handle(STAND_UP, "", 0.0)
    for t in (0.0, 0.3, 2.1):
        b.step(t)
    b.handle(BALANCE_STAND, "", 2.1)
    assert b.step(2.1).command == TROT_CMD
    assert b.step(2.7) is None and b.step(4.2) is None
    assert b.error_code == 1013
    return b


def resend_move(b: BridgeCore, parameter: str, start: float, end: float, clock=lambda t: t):
    """アプリと同じく Move を 10 Hz で送り直し、controller へ出た (now, 入力) を返す。"""
    sent = []
    t = start
    while t <= end + 1e-9:
        b.handle(MOVE, parameter, t, clock(t))
        i = b.step(t, clock(t))
        if i is not None:
            sent.append((t, i))
        t = round(t + 0.1, 2)
    return sent


def test_stand_then_walk_then_expire():
    """Move は controller が歩くに入って TROT_WAIT_S 秒（sim）たつまで速度 0 のまま待ち、それから
    0 から RELEASE_ACC で上がる（R22。Task I：一度に入れると跳ぶ）。送り直しが 1 秒無ければ止まる。"""
    b = BridgeCore(0.0)
    b.handle(STAND_UP, "", 0.0); assert b.step(0.0).command == STAND_CMD
    b.step(2.1); assert b.error_code == 1002
    b.handle(BALANCE_STAND, "", 2.1); assert b.step(2.1).command == TROT_CMD
    b.step(2.7); assert b.error_code == 1013
    # 歩くに入るのは 3.6（伏せ 1.8 秒 + 立つ 1.8 秒の後）。速度はその 0.5 秒後の 4.1 から上げる
    assert TROT_WAIT_S == 0.5
    sent = resend_move(b, '{"x":0.3,"y":0,"z":0}', 3.0, 4.8)
    assert b.error_code == 1015                               # 待っている間も窓口は「移動中」
    # 前後は ly（0.3 ÷ 0.4 = 0.75）。0.6 m/s² で 0.1 秒に 0.06 m/s = ly 0.15 ずつ。4.1 より前には出さない
    assert [t for t, _ in sent] == [4.2, 4.3, 4.4, 4.5, 4.6]
    assert [i.ly for _, i in sent] == pytest.approx([0.15, 0.30, 0.45, 0.60, 0.75])
    assert all(i.command == TROT_CMD and i.lx == 0 and i.rx == 0 for _, i in sent)
    assert sent[-1][1] == move_input(0.3, 0.0, 0.0)           # 着いたら Move の値そのもの
    assert b.step(5.7) is None and b.error_code == 1015       # 最後の送り直し（4.8）から 1 秒たつまでは歩く
    i = b.step(5.8); assert (i.lx, i.ly, i.rx) == (0, 0, 0) and b.error_code == 1013


def test_move_waits_for_trot_entry_in_sim_time_and_stop_cancels_it():
    """待つ 0.5 秒はシミュレーションの時間。待っている間に StopMove が来たら、速度は 1 通も出さない。"""
    half = lambda t: t / 2                                    # Gazebo が実時間の半分の速さ
    b = BridgeCore(0.0, 0.0)
    b.handle(STAND_UP, "", 0.0, 0.0); b.step(0.0, 0.0); b.step(0.4, 0.2)
    b.handle(BALANCE_STAND, "", 4.2, 2.1); assert b.step(4.2, 2.1).command == TROT_CMD
    sent = resend_move(b, '{"x":0,"y":0,"z":0.4}', 5.0, 9.5, half)   # sim で 2.5〜4.75
    # sim 3.6 + 0.5 = 4.1（now 8.2）から上げる。旋回 1.0 rad/s² は sim の 0.05 秒（now 0.1 秒）に rx 0.1
    assert [t for t, _ in sent] == pytest.approx([8.3, 8.4, 8.5, 8.6, 8.7, 8.8, 8.9, 9.0])
    assert [i.rx for _, i in sent] == pytest.approx([-0.1 * k for k in range(1, 9)])
    assert sent[-1][1].rx == pytest.approx(-0.8)

    b = BridgeCore(0.0)
    b.handle(STAND_UP, "", 0.0); b.step(0.0); b.step(0.2)
    b.handle(BALANCE_STAND, "", 2.1); b.step(2.1)
    assert resend_move(b, '{"x":0.3,"y":0,"z":0}', 3.0, 3.8) == []
    b.handle(STOP_MOVE, "", 3.9); assert b.error_code == 1013
    assert all(b.step(t) is None for t in (3.9, 4.1, 4.5, 6.0))


def test_move_waits_for_reported_trot_entry():
    """橋渡しのノードが controller の本当の切り替え（/rosout）を知らせるとき：待つ 0.5 秒は、知らせを
    受けた時刻から数える（controller の伏せ・立つは 1.8 秒より少し長くかかる）。知らせが来なければ、
    見積もりより TROT_REPORT_LATE_S 遅らせて入れる。"""
    def walking(reports_at):
        b = BridgeCore(0.0, reports_trot_entry=True)
        b.handle(STAND_UP, "", 0.0); b.step(0.0); b.step(0.2)
        b.handle(BALANCE_STAND, "", 2.1); b.step(2.1)
        sent, t = [], 3.0
        while t <= 6.0 + 1e-9:
            if reports_at is not None and abs(t - reports_at) < 1e-9:
                b.trot_entered(t)
            b.handle(MOVE, '{"x":0.3,"y":0,"z":0}', t)
            i = b.step(t)
            if i is not None:
                sent.append((t, i.ly))
            t = round(t + 0.1, 2)
        return sent, b.notes

    ramp = [pytest.approx(0.15 * k) for k in range(1, 6)]     # 0 から 0.6 m/s²（ly 0.15 / 0.1 秒）で 0.75 まで
    sent, _ = walking(reports_at=3.9)                         # 見積もり（3.6）より 0.3 秒遅く入った
    assert sent == list(zip([4.5, 4.6, 4.7, 4.8, 4.9], ramp))  # 3.9 + 0.5 = 4.4 から上げる
    sent, notes = walking(reports_at=None)
    assert TROT_REPORT_LATE_S == 1.0                          # 3.6 + 0.5 + 1.0 = 5.1 から上げる
    assert sent == list(zip([5.2, 5.3, 5.4, 5.5, 5.6], ramp))
    assert any("/rosout" in n for n in notes)

    b = BridgeCore(0.0, reports_trot_entry=True)              # 前の「歩く」の知らせは、次の「歩く」に使わない
    b.handle(STAND_UP, "", 0.0); b.step(0.0); b.step(0.2)
    b.handle(BALANCE_STAND, "", 2.1); b.step(2.1); b.trot_entered(3.7)
    b.handle(SIT, "", 5.0); b.step(5.0); b.step(5.2)
    b.handle(RISE_SIT, "", 10.0); assert b.step(10.0).command == STAND_CMD
    assert b.step(12.0).command == TROT_CMD                   # 4 は 2 の 2.0 秒後
    assert resend_move(b, '{"x":0.3,"y":0,"z":0}', 12.1, 13.0) == []   # まだ知らせが無い
    b.trot_entered(13.1)
    assert [t for t, _ in resend_move(b, '{"x":0.3,"y":0,"z":0}', 13.1, 13.8)] == [13.7, 13.8]   # 13.6 から上げる


def test_gesture_is_pretend_and_sit_uses_crouch():
    b = standing_core()
    b.handle(HELLO, "", 10.0); assert b.step(10.0) is None and "再現なし" in b.notes[-1]
    b.handle(SIT, "", 13.5); assert b.step(13.5).command == DOWN_CMD
    b.step(15.6); assert b.error_code == 1007 and state_from_error_code(1007) == SITTING


def test_invalid_command_ignored():
    b = BridgeCore(0.0); b.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 0.0)
    assert b.step(0.0) is None and b.error_code == 1001


def test_controller_gets_down_stand_trot_in_order():
    """脱力 → 伏せ → 立つ → 歩く：2 を 2 回、伏せの 1.8 秒が過ぎてから 4。続けて送らない。"""
    b = BridgeCore(0.0)
    sent = []
    t = 0.0
    b.handle(STAND_UP, "", t)
    while t < 6.0:
        if abs(t - 2.5) < 1e-9:
            b.handle(BALANCE_STAND, "", t)
        if abs(t - 3.5) < 1e-9:
            b.handle(MOVE, '{"x":0,"y":0,"z":0.25}', t)
        i = b.step(t)
        if i is not None:
            sent.append((round(t, 2), i))
        t = round(t + 0.05, 2)
    cmds = [(at, i.command) for at, i in sent]
    assert cmds[:3] == [(0.0, STAND_CMD), (0.2, STAND_CMD), (2.5, TROT_CMD)]
    # 3.5 の Move は、歩くに入った 3.6 から TROT_WAIT_S（0.5 秒）後の 4.1 から、1.0 rad/s² で上がる（R22）
    assert [at for at, _ in sent[3:8]] == [4.15, 4.2, 4.25, 4.3, 4.35]
    assert all(i.command == TROT_CMD for _, i in sent[3:8])
    assert [i.rx for _, i in sent[3:8]] == pytest.approx([-0.1, -0.2, -0.3, -0.4, -0.5])   # 左回り 0.25 rad/s → rx = −z / 0.5
    assert sent[8][0] == 4.5 and sent[8][1].rx == 0           # 送り直しが無いので 1 秒（3.5 + 1.0）で止まる。止めるのはすぐ
    assert all(i.ry == 0 for _, i in sent)


def released(b: BridgeCore, parameter: str, start: float, end: float, dt: float = 0.05, clock=lambda t: t):
    """橋渡しのノードと同じく 20 Hz で step し、アプリと同じく 10 Hz で Move を送り直す。出た (now, 入力)。"""
    sent, t, k = [], start, 0
    while t <= end + 1e-9:
        if k % 2 == 0:
            b.handle(MOVE, parameter, t, clock(t))
        i = b.step(t, clock(t))
        if i is not None:
            sent.append((t, i))
        t, k = round(t + dt, 3), k + 1
    return sent


def test_held_turn_is_ramped_in_at_the_executor_acceleration():
    """Task I（FL1〜FL3・FN1）：立った直後、最初の旋回の Move は歩くに入る前に届き、実行器は待つ間に
    0.5 rad/s まで上げ終える。橋渡しはそれを一度に入れず（rx 0 → −1.00 の跳び。FL3 はこれで転んだ）、
    GridExecutor と同じ加速度の上限で 0 から上げる。"""
    assert RELEASE_ACC == GRID_EXEC_CONFIG.acc == (0.6, 0.6, 1.0)
    b = BridgeCore(0.0)
    b.handle(STAND_UP, "", 0.0); b.step(0.0); b.step(0.2); b.step(2.1)
    b.handle(BALANCE_STAND, "", 2.1); assert b.step(2.1).command == TROT_CMD
    sent = released(b, '{"x":0,"y":0,"z":-0.5}', 3.0, 5.0)   # 右回り 0.5 rad/s（rx +1.0）。歩けるのは 4.1 から
    ts, rxs = [t for t, _ in sent], [i.rx for _, i in sent]
    assert ts[0] > 4.1 and rxs[-1] == 1.0                     # 4.1 より前には出さず、最後は Move の値そのもの
    prev_t, prev_rx = 4.1, 0.0
    for t, rx in zip(ts, rxs):                                # 上げ方は 1.0 rad/s² = rx 2.0 / 秒まで
        assert rx - prev_rx <= RELEASE_ACC[2] / W_Z_MAX * (t - prev_t) + 1e-9
        prev_t, prev_rx = t, rx
    assert rxs == sorted(rxs) and len(sent) == 10             # 0.05 秒ごとに 0.1 ずつ、0.5 秒かけて
    assert ts[-1] == pytest.approx(4.6)
    assert all(b.step(t) is None for t in (4.65, 4.7))        # 着いた後、同じ値の送り直しは出さない


def test_held_walk_with_steering_ramps_each_axis_at_its_limit():
    """前後・左右・旋回は、それぞれ 0.6 m/s²・0.6 m/s²・1.0 rad/s² で上がる（controller の倍率で直して）。"""
    b = BridgeCore(0.0)
    b.handle(STAND_UP, "", 0.0); b.step(0.0); b.step(0.2); b.step(2.1)
    b.handle(BALANCE_STAND, "", 2.1); b.step(2.1)
    sent = released(b, '{"x":0.3,"y":-0.15,"z":0.1}', 3.0, 5.0)
    first = sent[0][1]
    assert sent[0][0] == pytest.approx(4.15)
    assert (first.ly, first.lx, first.rx) == pytest.approx((0.6 * 0.05 / V_X_MAX, 0.6 * 0.05 / V_Y_MAX,
                                                            -1.0 * 0.05 / W_Z_MAX))
    last = sent[-1][1]
    assert (last.ly, last.lx, last.rx) == pytest.approx((0.75, 0.5, -0.2)) and last == move_input(0.3, -0.15, 0.1)


def test_stop_or_slowing_during_the_ramp_is_at_once():
    """上げる途中の StopMove は次の step ですぐ 0。遅くする（0 の方へ戻す）のも、軸ごとに上げ方を待たない。
    controller の速度が Move に追いついたら上げ方は終わり、その後の変化は実行器が自分で上げたもの。"""
    def ramping(parameter):
        b = BridgeCore(0.0)
        b.handle(STAND_UP, "", 0.0); b.step(0.0); b.step(0.2); b.step(2.1)
        b.handle(BALANCE_STAND, "", 2.1); b.step(2.1)
        return b, released(b, parameter, 3.0, 4.3)

    b, sent = ramping('{"x":0,"y":0,"z":0.5}')
    assert [i.rx for _, i in sent] == pytest.approx([-0.1, -0.2, -0.3, -0.4])
    b.handle(STOP_MOVE, "", 4.3)
    i = b.step(4.35); assert (i.lx, i.ly, i.rx) == (0, 0, 0) and b.error_code == 1013
    assert b.step(4.4) is None

    b, sent = ramping('{"x":0.3,"y":0,"z":0.5}')
    assert [i.ly for _, i in sent] == pytest.approx([0.075 * k for k in range(1, 5)])
    assert [i.rx for _, i in sent] == pytest.approx([-0.1 * k for k in range(1, 5)])
    b.handle(MOVE, '{"x":0.3,"y":0,"z":0.1}', 4.3)            # 旋回だけ、上げた値（rx −0.4）より遅く
    i = b.step(4.35); assert (i.ly, i.rx) == pytest.approx((0.375, -0.2))   # 旋回はすぐ、前後は上げる途中
    for t in (4.4, 4.45, 4.5, 4.55, 4.6):
        i = b.step(t)
    assert i == move_input(0.3, 0.0, 0.1)                     # 追いついた（4.6）
    b.handle(MOVE, '{"x":0.3,"y":0,"z":0.2}', 4.7)            # 追いついた後は、そのまま入る
    assert b.step(4.7).rx == pytest.approx(-0.4)


def test_a_move_after_the_walk_began_is_not_ramped_again():
    """歩くに入ってしばらくたってから来た Move（実行器が自分で 0 から上げる）は、今までどおりそのまま入る。
    上げ終えた後の速度の変化もそのまま。伏せてまた歩くに入ったら、また 0 から上げる。"""
    b = standing_core()                                       # 歩けるのは 4.1 から
    b.handle(MOVE, '{"x":0.06,"y":0,"z":0}', 5.0); assert b.step(5.0).ly == pytest.approx(0.15)
    b.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 5.1); assert b.step(5.1).ly == pytest.approx(0.75)
    b.handle(MOVE, '{"x":0,"y":0,"z":0.5}', 5.2); assert b.step(5.2).rx == pytest.approx(-1.0)
    b.handle(SIT, "", 5.3); assert b.step(5.3).rx == 0         # 止めてから伏せる
    assert b.step(5.5).command == DOWN_CMD
    b.handle(RISE_SIT, "", 10.0); assert b.step(10.0).command == STAND_CMD
    assert b.step(12.0).command == TROT_CMD                   # 歩くに入るのは 12.0、歩けるのは 12.5 から
    sent = released(b, '{"x":0.3,"y":0,"z":0}', 12.0, 12.75)
    assert [t for t, _ in sent] == pytest.approx([12.55, 12.6, 12.65, 12.7, 12.75])
    assert [i.ly for _, i in sent] == pytest.approx([0.075, 0.15, 0.225, 0.3, 0.375])


def test_slow_gazebo_keeps_app_timing_and_command_order():
    """Gazebo が実時間の半分の速さ（sim = now / 2）。窓口の時間は now、command の間隔は sim で数える。"""
    b = BridgeCore(0.0, 0.0)
    sent = []
    t = 0.0
    b.handle(STAND_UP, "", t, t / 2)
    while t < 8.0:
        if abs(t - 3.0) < 1e-9:                      # アプリは StandUp の 3 秒後（実際の時間）に送る
            assert b.error_code == 1002
            b.handle(BALANCE_STAND, "", t, t / 2)
        i = b.step(t, t / 2)
        if i is not None:
            sent.append((t / 2, i.command))
        t = round(t + 0.05, 2)
    assert b.error_code == 1013
    assert [c for _, c in sent] == [STAND_CMD, STAND_CMD, TROT_CMD]
    assert sent[2][0] >= 1.8 + 0.2 - 1e-9              # 4 は 2 通目の 2 が効いた（伏せの 1.8 秒）後


def test_move_input_follows_controller_table():
    i = move_input(0.3, 0.15, 0.8)
    assert i.command == TROT_CMD
    assert (i.ly, i.lx, i.rx) == pytest.approx((0.75, -0.5, -1.0))   # 旋回は上限 0.5 rad/s で切る
    assert move_input(-1.0, -1.0, 0.0).ly == -1.0 and move_input(-1.0, -1.0, 0.0).lx == 1.0


def test_stop_and_bad_move_parameters():
    b = standing_core()
    b.handle(MOVE, '{"x":NaN,"y":0,"z":0}', 5.0)
    assert b.step(5.0) is None and b.error_code == 1013
    b.handle(MOVE, '{"x":-0.2}', 5.1); assert b.step(5.1).ly == pytest.approx(-0.5)
    b.handle(MOVE, '{"x":-0.2}', 5.2); assert b.step(5.2) is None      # 同じ速度の送り直しは出さない
    b.handle(STOP_MOVE, "", 5.3); i = b.step(5.3)
    assert i.command == TROT_CMD and (i.lx, i.ly, i.rx) == (0, 0, 0) and b.error_code == 1013
    b.handle(STOP_MOVE, "", 5.4); assert b.step(5.4) is None


# ---- 転んだとき（Ruling R38） ----
UPRIGHT = (0.02, -0.01, 0.33)    # 立っているときの (roll, pitch, 胴の高さ)


def fell(b: BridgeCore) -> bool:
    return any("（Gazebo で転びました）" in n for n in b.notes)


@pytest.mark.parametrize("roll,pitch,height", [
    (1.2, 0.1, 0.20),       # 横に倒れた
    (-0.3, -1.1, 0.25),     # 前に倒れた
    (0.1, 0.0, 0.12),       # 傾きは小さいが、胴が落ちた
])
def test_fall_while_walking_reports_lying_and_stops(roll, pitch, height):
    """立って歩いているつもりで |roll| か |pitch| が 1.0 rad を超えるか、胴が 0.15 m 未満になったら、
    error_code を 1001（伏せ扱い）にし、速度を 0 にして、ノートに書く。"""
    assert (FALL_TILT, FALL_Z) == (1.0, 0.15)
    b = standing_core()
    b.observe(*UPRIGHT, 4.2)
    assert [t for t, _ in resend_move(b, '{"x":0.3,"y":0,"z":0}', 4.3, 4.5)] == [4.3, 4.4, 4.5]   # 0 から上げる途中
    assert b.error_code == 1015
    b.observe(roll, pitch, height, 4.6)
    assert b.error_code == 1001 and state_from_error_code(b.error_code) == LYING
    assert fell(b)
    i = b.step(4.6)
    assert i.command == TROT_CMD and (i.lx, i.ly, i.rx) == (0, 0, 0)


def test_after_a_fall_nothing_stands_it_up():
    """起き上がらせない（利用者が Gazebo を起動し直す）。StopMove のほかは受け付けず、体が起きても 1001 のまま。"""
    b = standing_core()
    b.observe(*UPRIGHT, 4.2)
    b.observe(0.0, 1.3, 0.22, 4.3)
    assert b.error_code == 1001
    b.notes.clear()
    b.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 4.4)
    b.handle(STAND_UP, "", 5.0)
    b.handle(BALANCE_STAND, "", 8.0)
    b.handle(STOP_MOVE, "", 8.5)
    assert all(b.step(t) is None for t in (4.4, 5.0, 7.1, 8.0, 8.5, 10.0))
    assert len(b.notes) == 3 and all("Gazebo を起動し直して" in n for n in b.notes)
    b.observe(*UPRIGHT, 10.0)                                 # 自分で起きても、伏せ扱いのまま
    assert b.error_code == 1001


def test_fall_during_a_gesture_is_not_undone_when_it_ends():
    b = standing_core()
    b.observe(*UPRIGHT, 4.2)
    b.handle(HELLO, "", 5.0)                                   # 3 秒後に 1013 に戻る予定
    b.observe(1.4, 0.2, 0.18, 6.0)
    b.step(8.5)
    assert b.error_code == 1001 and fell(b)


def test_normal_sway_is_not_a_fall():
    """普通に揺れる程度（0.6 rad）や、歩き出しで胴が少し下がる（0.24 m）程度では変わらない。境目ちょうども転びではない。"""
    b = standing_core()
    b.observe(*UPRIGHT, 4.2)
    resend_move(b, '{"x":0.3,"y":0,"z":0}', 4.3, 4.5)
    for t, (roll, pitch, height) in zip((4.6, 4.7, 4.8, 4.9, 5.0),
                                        [(0.6, 0.0, 0.30), (-0.6, 0.1, 0.30), (0.1, 0.6, 0.24),
                                         (FALL_TILT, -FALL_TILT, 0.30), (0.0, 0.0, FALL_Z)]):
        b.observe(roll, pitch, height, t)
    assert b.error_code == 1015 and not fell(b)


def test_standing_up_is_not_a_fall():
    """窓口は、体が立ち上がりきる前に 1002 になる。胴が UPRIGHT_Z まで上がってから見張る。"""
    b = BridgeCore(0.0)
    b.handle(STAND_UP, "", 0.0); b.step(0.0); b.step(0.2)
    b.observe(0.0, 0.0, 0.068, 1.0)                           # 脱力で伏せている（1001）
    b.step(2.1); assert b.error_code == 1002
    b.observe(0.0, 0.05, 0.10, 2.1)                           # 1002 だが、胴はまだ低い
    b.observe(0.0, 0.02, 0.20, 2.8)
    assert b.error_code == 1002 and not fell(b)
    assert UPRIGHT_Z == 0.25
    b.observe(*UPRIGHT, 3.6)                                  # 立ち上がった。ここから見張る
    b.observe(0.0, 0.0, 0.10, 3.7)
    assert b.error_code == 1001 and fell(b)


@pytest.mark.parametrize("api_id,code", [(STAND_DOWN, 1004), (SIT, 1007)])
def test_lying_down_or_sitting_is_not_a_fall(api_id, code):
    """伏せ・おすわり（Gazebo では伏せ）の間、窓口はまだ 1013 だが胴は下がる。下りていく間とその後は見ない。"""
    b = standing_core()
    b.observe(*UPRIGHT, 4.2)
    b.handle(api_id, "", 5.0); b.step(5.0)
    b.observe(0.0, 0.1, 0.12, 5.8)
    b.step(7.1); assert b.error_code == code
    b.observe(1.3, 0.0, 0.10, 7.2)
    assert b.error_code == code and not fell(b)


def test_self_points_dropped_and_mount_matches_perception():
    cfg = PerceptionConfig()
    assert LIDAR_IN_BODY == (cfg.lidar_x, 0.0, cfg.lidar_z) and cfg.lidar_pitch == 0.0
    joint = re.search(r'<joint name="utlidar_joint".*?<origin xyz="([^"]+)" rpy="([^"]+)"',
                      XACRO.read_text(encoding="utf-8"), re.S)
    assert [float(v) for v in joint.group(1).split()] == list(LIDAR_IN_BODY)
    assert [float(v) for v in joint.group(2).split()] == [0.0, 0.0, 0.0]
    pts = np.array([[-0.1, 0.0, -0.3],        # 自分の脚（機体座標 x 0.18、z −0.2）
                    [-0.6, 0.1, -0.2],        # 自分の後ろ足（機体 x −0.32）
                    [0.8, 0.0, -0.2],         # 前の箱（機体 x 1.08）
                    [math.inf, 0.0, 0.0]])    # 当たらなかった本
    assert drop_self_points(pts).tolist() == [[0.8, 0.0, -0.2]]


def quat_rpy(roll: float, pitch: float, yaw: float):
    """(w, x, y, z)。z-y-x の順（yaw・pitch・roll）。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def test_ground_points_dropped_while_lying():
    """伏せ（base の高さ 0.068 m、水平）：LiDAR は地面から 0.168 m。地面の点（世界の z < 0.05）は除き、
    低い箱の点は残す（R23）。"""
    assert GROUND_Z == 0.05
    h = 0.068 + LIDAR_IN_BODY[2]
    pts = np.array([[0.30, 0.25, -h],          # 横の地面（機体の外形の外で、前方の箱の中に入る所）
                    [0.60, 0.0, -h + 0.01],    # 前の地面（世界の z 0.01）
                    [0.80, 0.0, -h + 0.06],    # 前の低い箱（世界の z 0.06）
                    [0.80, 0.0, -0.05],        # 前の箱
                    [-0.1, 0.0, -0.05],        # 自分の体（機体の外形の中）
                    [math.nan, 0.0, 0.0]])
    kept = drop_ground_and_self_points(pts, 0.068, (1.0, 0.0, 0.0, 0.0))
    np.testing.assert_allclose(kept, [[0.80, 0.0, -h + 0.06], [0.80, 0.0, -0.05]])
    turned = drop_ground_and_self_points(pts, 0.068, quat_rpy(0.0, 0.0, 1.2))   # 向き（yaw）は関係ない
    np.testing.assert_allclose(turned, kept)


def test_ground_uses_body_tilt():
    """前に 0.3 rad 傾いた（pitch +0.3、鼻が下）とき、傾きを入れて世界の z を出す。"""
    q = quat_rpy(0.0, 0.3, 0.0)
    body = np.array([[1.0, 0.0, -0.05], [1.0, 0.0, 0.20]])     # 機体の座標
    world_z = 0.30 - math.sin(0.3) * body[:, 0] + math.cos(0.3) * body[:, 2]
    assert world_z[0] < GROUND_Z < world_z[1]                 # 1 点目は地面（傾きを無視すると 0.25）
    kept = drop_ground_and_self_points(body - np.asarray(LIDAR_IN_BODY), 0.30, q)
    np.testing.assert_allclose(kept, [body[1] - np.asarray(LIDAR_IN_BODY)])
    assert drop_ground_and_self_points(np.empty((0, 3)), 0.30, q).shape == (0, 3)


def test_boxes_follow_heading_and_collision():
    w = BoxWorld()
    name, box = w.place("front", x=1.0, y=2.0, yaw=math.pi / 2)
    assert (box.x, box.y) == pytest.approx((1.0, 3.2)) and box.size == 0.3
    w.add(name, box)
    _, back = w.place("back", x=0.0, y=0.0, yaw=0.0)
    assert back.x == pytest.approx(-1.2)
    assert w.world(1.0, 2.0) == {"boxes": [[1.0, 3.2, 0.3]], "collided": False}
    assert w.world(1.0, 3.2 - 0.15 - FOOTPRINT + 0.01)["collided"] is True
    assert w.clear() == [name] and w.world(1.0, 3.0) == {"boxes": [], "collided": False}
