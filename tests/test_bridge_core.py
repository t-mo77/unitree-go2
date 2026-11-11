import math
import re
from pathlib import Path

import numpy as np
import pytest

from go2_nl_control.actions import (BALANCE_STAND, HELLO, LYING, MOVE, RISE_SIT, SIT, SITTING,
                                    STAND_DOWN, STAND_UP, STOP_MOVE, state_from_error_code)
from go2_nl_control.perception import PerceptionConfig
from gazebo.bridge_core import (DOWN_CMD, FALL_TILT, FALL_Z, FIXED_DOWN, FIXED_STAND, GROUND_Z,
                                LIDAR_IN_BODY, MAX_SENDS, PASSIVE, RELEASE_ACC, RESEND_S, STAND_CMD,
                                STANDING_CODES, TROT_CMD, TROT_WAIT_S, TROTTING, UPRIGHT_Z, V_X_MAX,
                                V_Y_MAX, W_Z_MAX, BoxWorld, BridgeCore, ControllerInput,
                                drop_ground_and_self_points, drop_self_points, move_input, parse_switch)
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
    """橋渡しのノードが controller の切り替え（/rosout）を知らせるとき：Move の速度は、歩くに入った知らせを
    受けてから TROT_WAIT_S 秒（sim）たって 0 から上げる。知らせが来なければ速度は入れない（見積もりで入れると、
    伏せたままの controller に速度を送ることになる。2026-11-11 の件）。4 を送り直し、MAX_SENDS 回でも来なければ
    あきらめてノートに書き、窓口は立ち固め（1002）にする（アプリは BalanceStand からやり直す）。
    前の「歩く」の知らせを次の「歩く」に使わないことは test_sit_and_rise_follow_the_reports。"""
    walk = '{"x":0.3,"y":0,"z":0}'
    late = Rig(FakeController(report_delay=0.3))              # 知らせはどれも 0.3 秒遅れて届く
    late.run(6.0, api=[(0.0, STAND_UP), (3.0, BALANCE_STAND)], move=(4.0, 6.0, walk))
    trot_known = 5.05                                         # 歩くに入ったのは 4.705、知らせは次の周期の 5.05
    assert late.ctl.entered_at(TROTTING) == pytest.approx(4.705)
    sent = [(t, i.ly) for t, i, _ in late.sent if i.ly != 0.0]
    assert [t for t, _ in sent] == pytest.approx([trot_known + TROT_WAIT_S + 0.05 * k for k in range(1, 10)])
    assert [ly for _, ly in sent] == pytest.approx([0.075 * k for k in range(1, 10)])

    lost = Rig(FakeController(lose_report=lambda k: k == 2))  # 「歩くに入った」の知らせだけ落ちる
    lost.run(13.0, api=[(0.0, STAND_UP), (3.0, BALANCE_STAND)], move=(4.0, 13.0, walk))
    assert lost.ctl.state == TROTTING
    assert all((i.lx, i.ly, i.rx) == (0.0, 0.0, 0.0) for _, i, _ in lost.sent)   # 速度は 1 通も送らない
    assert RESEND_S == 2.8 and MAX_SENDS == 3                 # 4 を 4.1 に送り、2.8 秒ごとに 2 回送り直す
    assert lost.commands() == [(0.0, STAND_CMD), (2.05, STAND_CMD), (4.1, TROT_CMD), (6.9, TROT_CMD),
                               (9.7, TROT_CMD)]
    assert sum("送り直します" in n for n in lost.b.notes) == MAX_SENDS - 1
    assert any("/rosout" in n for n in lost.b.notes)
    assert any("Gazebo を起動し直して" in n for n in lost.b.notes) and lost.b.error_code == 1002


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


# ---- controller の状態の切り替え（2026-11-11：「立って」が終わったのに伏せたまま） ----
CYCLE = 0.005                                   # controller の 1 周期（シミュレーションの時間）
_FSM = {PASSIVE: {2: FIXED_DOWN},               # 各状態の checkChange（unitree_guide controller）
        FIXED_DOWN: {1: PASSIVE, 2: FIXED_STAND},
        FIXED_STAND: {1: PASSIVE, 2: FIXED_DOWN, 4: TROTTING},
        TROTTING: {1: PASSIVE, 2: FIXED_STAND}}


class FakeController:
    """unitree_guide controller の状態の切り替えだけの真似（UnitreeGuideController::update と各状態の checkChange）。

    - /control_input の command は次の通が来るまで残り、状態に入る（enter）と 0 に戻る。
    - 伏せ・立つは、入って hold 秒（既定 1.8 = percent 1.5 × 1.2 秒）たつまで次へ行かない。その後も、
      伏せは 1・2 のほか（4 も）無視する。
    - 切り替える周期に "Switched from a to b" を出し（report_delay 秒後に橋渡しへ届く）、次の周期に enter する。
    - lose_input(n)：n 通目（0 から）の入力が届かない。lose_report(k)：k 個目の切り替えの知らせが届かない。"""

    def __init__(self, hold=None, report_delay=0.0, lose_input=lambda n: False,
                 lose_report=lambda k: False, state=PASSIVE, command=0, entered=0.0):
        self.hold = {FIXED_DOWN: 1.8, FIXED_STAND: 1.8, **(hold or {})}
        self.report_delay, self.lose_input, self.lose_report = report_delay, lose_input, lose_report
        self.state, self.command, self.entered = state, command, entered
        self.t = 0.0
        self.switches = []      # (t, a, b)
        self.inputs = []        # 送られた入力（届かなかったものも）(t, ControllerInput)
        self._next = None
        self._reports = []      # (橋渡しに届く t, a, b)

    def receive(self, i, t):
        n = len(self.inputs)
        self.inputs.append((t, i))
        if not self.lose_input(n):
            self.command = i.command

    def advance(self, t):
        while self.t + CYCLE <= t + 1e-9:
            self.t = round(self.t + CYCLE, 6)
            if self._next is not None:                       # exit → enter（command は 0 に戻る）
                self.state, self._next = self._next, None
                self.entered, self.command = self.t, 0
                continue
            held = self.t - self.entered >= self.hold.get(self.state, 0.0) - 1e-9
            nxt = _FSM[self.state].get(self.command, self.state) if held else self.state
            if nxt != self.state:                            # ログを出すのは enter の 1 周期前
                self._next = nxt
                if not self.lose_report(len(self.switches)):
                    self._reports.append((self.t + self.report_delay, self.state, nxt))
                self.switches.append((self.t, self.state, nxt))

    def reports(self, t):
        due = [(a, b) for at, a, b in self._reports if at <= t + 1e-9]
        self._reports = [r for r in self._reports if r[0] > t + 1e-9]
        return due

    def entered_at(self, state):
        """最後に state へ切り替わった時刻（ログを出した周期、sim）。"""
        return max(t for t, _, b in self.switches if b == state)


class Rig:
    """BridgeCore と FakeController を、橋渡しのノードと同じく 20 Hz（実際の時間）で回す。
    clock：実際の時間 → シミュレーションの時間。reports：controller の切り替えを橋渡しに知らせる
    （ノードは /rosout で知る）。"""

    def __init__(self, ctl=None, reports=True, clock=lambda t: t):
        self.ctl = ctl or FakeController()
        self.clock, self.reports = clock, reports
        self.b = (BridgeCore(0.0, clock(0.0), reports_switches=True) if reports
                  else BridgeCore(0.0, clock(0.0)))
        self.t = 0.0
        self.sent = []          # 橋渡しが送った (t, 入力, そのときの controller の本当の状態)
        self.trace = []         # 毎周期の (t, error_code, controller の本当の状態)

    def run(self, until, api=(), move=None):
        """api：[(実際の時刻, api_id)]。move：(始め, 終わり, parameter)。Move はアプリと同じく 10 Hz で送り直す。"""
        api = {round(t, 3): a for t, a in api}
        while self.t <= until + 1e-9:
            t, s = self.t, self.clock(self.t)
            self.ctl.advance(s)
            for a, b in self.ctl.reports(s):
                if self.reports:
                    self.b.controller_switched(b, s, from_state=a)
            if t in api:
                self.b.handle(api[t], "", t, s)
            if move is not None and move[0] - 1e-9 <= t <= move[1] + 1e-9 and round(t * 20) % 2 == 0:
                self.b.handle(MOVE, move[2], t, s)
            i = self.b.step(t, s)
            if i is not None:
                self.sent.append((t, i, self.ctl.state))
                self.ctl.receive(i, s)
            self.trace.append((t, self.b.error_code, self.ctl.state))
            self.t = round(t + 0.05, 3)
        return self

    def commands(self):
        """速度 0 の入力（状態を変える command と、止める入力）の (t, command)。"""
        return [(t, i.command) for t, i, _ in self.sent if (i.lx, i.ly, i.rx) == (0.0, 0.0, 0.0)]

    def first_velocity(self):
        """速度を入れた最初の入力の時刻（実際の時間）。"""
        return next(t for t, i, _ in self.sent if (i.lx, i.ly, i.rx) != (0.0, 0.0, 0.0))

    def code_at(self, t):
        return next(c for at, c, _ in self.trace if abs(at - t) < 1e-9)


def stuck(rig: Rig) -> bool:
    """2026-11-11 の Gazebo：controller は伏せ（fixed down）で、残っている command が 4（伏せは無視し続ける）。"""
    return rig.ctl.state == FIXED_DOWN and rig.ctl.command == TROT_CMD


SLOW = [   # (clock, BalanceStand の時刻)。どちらも controller の伏せが 2.4 秒（見積もりは 1.8 秒）かかる
    pytest.param(lambda t: t, 2.1, id="rtf1"),
    pytest.param(lambda t: t / 2, 3.0, id="rtf0.5-app"),   # アプリと同じく StandUp の 3 秒後（実際の時間）
]


def test_fake_controller_agrees_with_the_estimate():
    """FakeController は、見積もり（伏せ 1.8 秒 + 立つ 1.8 秒）どおりの controller なら 3.6 秒で歩くに入る。"""
    rig = Rig(reports=False).run(5.0, api=[(0.0, STAND_UP), (2.1, BALANCE_STAND)])
    assert rig.commands() == [(0.0, STAND_CMD), (0.2, STAND_CMD), (2.1, TROT_CMD)]
    assert [b for _, _, b in rig.ctl.switches] == [FIXED_DOWN, FIXED_STAND, TROTTING]
    assert rig.ctl.entered_at(TROTTING) == pytest.approx(3.6, abs=0.02)


@pytest.mark.parametrize("clock,balance_at", SLOW)
def test_open_loop_sequencing_leaves_a_slow_controller_lying(clock, balance_at):
    """知らせ無し（テストと偽の道）は今までどおり見積もりで送る：2・2 の後、伏せの 1.8 秒 + 0.2 秒で 4。
    controller の伏せが 2.4 秒かかると、4 が残っていた 2 を上書きし、controller は伏せのまま 4 を無視し続ける
    （2026-11-11 の件）。窓口は立って歩けるつもり（1013）のまま。"""
    rig = Rig(FakeController(hold={FIXED_DOWN: 2.4}), reports=False, clock=clock)
    rig.run(12.0, api=[(0.0, STAND_UP), (balance_at, BALANCE_STAND)])
    assert [(round(clock(t), 6), c) for t, c in rig.commands()] == [
        (0.0, STAND_CMD), (0.2, STAND_CMD), (max(clock(balance_at), 2.0), TROT_CMD)]
    assert [b for _, _, b in rig.ctl.switches] == [FIXED_DOWN]
    assert stuck(rig) and rig.b.error_code == 1013


# ---- 知らせ有り：確かめた状態からだけ送る（Gazebo の橋渡しのノード） ----
def assert_honest(rig: Rig, window: float = 0.1) -> None:
    """controller が脱力・伏せに window 秒（sim）より長くいる間、error_code は立ち・歩き（1002・1013・1015）でない。"""
    since = None
    for t, code, state in rig.trace:
        s = rig.clock(t)
        if state not in (PASSIVE, FIXED_DOWN):
            since = None
            continue
        since = s if since is None else since
        assert not (code in STANDING_CODES and s - since > window + 1e-9), (t, code, state)


def no_trot_while_lying(rig: Rig) -> bool:
    """4（速度の入力も 4）を、controller が本当に脱力・伏せにいる間に送っていない。"""
    return all(state not in (PASSIVE, FIXED_DOWN) for _, i, state in rig.sent if i.command == TROT_CMD)


def test_closed_loop_stand_and_walk():
    """伏せ・立つ・歩くの command は、確かめた状態に入った知らせから HOLD_S + SETTLE_S（2.0 秒）後に 1 通ずつ。
    StandUp の 2 秒（実際の時間）で窓口は 1002 になるが、controller が立つに入ったと知らされるまでは 1001 のまま。
    速度は、歩くに入った知らせ（4.15）から TROT_WAIT_S 後に 0 から上げる。"""
    rig = Rig().run(7.0, api=[(0.0, STAND_UP), (3.0, BALANCE_STAND)], move=(4.0, 6.0, '{"x":0.3,"y":0,"z":0}'))
    assert [b for _, _, b in rig.ctl.switches] == [FIXED_DOWN, FIXED_STAND, TROTTING]
    # 伏せの知らせは 0.05、立つは 2.10、歩くは 4.15。7.0 の 4 は、Move の送り直しが止まって 1 秒後の「止める」
    assert rig.commands() == [(0.0, STAND_CMD), (2.05, STAND_CMD), (4.1, TROT_CMD), (7.0, TROT_CMD)]
    assert [rig.code_at(t) for t in (1.95, 2.0, 2.05, 2.1, 3.45, 3.5, 4.0)] == [1001] * 3 + [1002] * 2 + [1013, 1015]
    assert rig.first_velocity() == pytest.approx(4.15 + TROT_WAIT_S + 0.05)
    assert rig.b.notes == []                                  # 普通に立てたときは何も書かない
    assert no_trot_while_lying(rig)
    assert_honest(rig)


@pytest.mark.parametrize("clock,balance_at", SLOW)
def test_slow_controller_is_followed_to_trotting(clock, balance_at):
    """controller の伏せが見積もりより長い（2.4 秒）。見積もりで送ると伏せのまま残る（上の
    test_open_loop_sequencing_leaves_a_slow_controller_lying）。知らせ有りなら、伏せの間は 2 しか送らず、
    立つに入ったと知らされてから 4 を送るので、歩くに入る。速度は歩くに入った後にだけ送る。"""
    rig = Rig(FakeController(hold={FIXED_DOWN: 2.4}), clock=clock)
    rig.run(14.0, api=[(0.0, STAND_UP), (balance_at, BALANCE_STAND)],
            move=(balance_at + 1.0, 13.0, '{"x":0.3,"y":0,"z":0}'))
    assert not stuck(rig) and rig.ctl.state == TROTTING
    assert [b for _, _, b in rig.ctl.switches] == [FIXED_DOWN, FIXED_STAND, TROTTING]
    assert no_trot_while_lying(rig)
    assert clock(rig.first_velocity()) >= rig.ctl.entered_at(TROTTING) + TROT_WAIT_S
    assert_honest(rig)


def test_error_code_waits_for_the_controller_to_stand():
    """Gazebo が実時間の半分の速さで、controller の伏せも 2.4 秒かかる。StandUp の 2 秒（実際の時間）がたっても
    controller は伏せなので 1001 のまま（アプリには、まだ立っていないと見える）。アプリが StandUp の 3 秒後に送る
    BalanceStand は受け付け（立つ途中）、controller が立つに入ったと知らされたら 1013 にする。
    MASK_NOTE_S（sim）より長く待たせたら、ノートに書く。"""
    rig = Rig(FakeController(hold={FIXED_DOWN: 2.4}), clock=lambda t: t / 2)
    rig.run(4.0, api=[(0.0, STAND_UP), (3.0, BALANCE_STAND)])
    assert {c for _, c, _ in rig.trace} == {1001}
    assert not any("受け付けません" in n for n in rig.b.notes)
    assert any("fixed down のままなので" in n and "/rosout" in n for n in rig.b.notes)
    rig.run(6.0)
    stood = rig.ctl.entered_at(FIXED_STAND)                   # sim 2.41（実際の時間 4.82）。知らせは次の周期の 4.85
    assert stood == pytest.approx(2.41)
    assert rig.code_at(4.8) == 1001 and rig.code_at(4.85) == 1013
    assert_honest(rig)


def test_a_bridge_restarted_on_the_stuck_controller_stands_it_up():
    """2026-11-11 の Gazebo のまま（controller は伏せで 4 が残っている）、橋渡しだけ起動し直した。橋渡しは脱力の
    つもりで 2 を送り、controller は伏せから立つへ行く（2 が 4 を上書きする）。知らせは「伏せ」ではなく「立つ」
    なので、そこから続ける。"""
    rig = Rig(FakeController(state=FIXED_DOWN, command=TROT_CMD, entered=-10.0))
    rig.run(7.0, api=[(0.0, STAND_UP), (3.0, BALANCE_STAND)], move=(4.0, 7.0, '{"x":0.3,"y":0,"z":0}'))
    assert [(a, b) for _, a, b in rig.ctl.switches] == [(FIXED_DOWN, FIXED_STAND), (FIXED_STAND, TROTTING)]
    assert rig.commands()[:2] == [(0.0, STAND_CMD), (3.0, TROT_CMD)]
    assert any("fixed stand に切り替わりました（fixed down のはずでした）" in n for n in rig.b.notes)
    assert rig.code_at(1.95) == 1001 and rig.code_at(2.0) == 1002
    assert rig.first_velocity() >= rig.ctl.entered_at(TROTTING) + TROT_WAIT_S
    assert no_trot_while_lying(rig)
    assert_honest(rig)


def test_the_incident_state_met_while_walking_recovers_at_once():
    """立って歩いているつもりのところで、ほかの送り手が controller を 2・2 で伏せさせ、伏せの間に 4 を送った
    （2026-11-11 と同じ：伏せで 4 が残る）。知らせで伏せと分かったら窓口は 1001 にし、行きたい状態（歩く）が
    あるので、すぐ伏せから 2 → 立つ → 4 で戻す。2 は残っていた 4 を上書きする。"""
    rig = Rig().run(6.0, api=[(0.0, STAND_UP), (3.0, BALANCE_STAND)], move=(4.0, 6.0, '{"x":0.3,"y":0,"z":0}'))
    assert rig.ctl.state == TROTTING and rig.code_at(6.0) == 1015
    rig.ctl.receive(ControllerInput(STAND_CMD), 6.0)          # 歩く → 立つ
    rig.run(6.2)
    rig.ctl.receive(ControllerInput(STAND_CMD), 6.2)          # 立つの 1.8 秒の後（7.81）に伏せへ
    rig.run(8.0)
    rig.ctl.receive(ControllerInput(TROT_CMD), 8.0)           # 伏せの間に 4：伏せは無視し続ける
    rig.run(9.5)
    assert stuck(rig) and rig.b.error_code == 1001
    assert any("trotting から fixed stand に切り替わりました（橋渡しが待っていた切り替えではありません）" in n
               for n in rig.b.notes)
    rig.run(14.0, move=(12.0, 13.5, '{"x":0.3,"y":0,"z":0}'))
    assert rig.ctl.state == TROTTING
    later = [(t, c) for t, c in rig.commands() if t > 6.0]
    assert later == [(9.85, STAND_CMD), (11.9, TROT_CMD)]   # 伏せの知らせ 7.85 + 2.0、立つ 9.90 + 2.0
    assert rig.code_at(9.85) == 1001 and rig.code_at(9.9) == 1013
    assert min(t for t, i, _ in rig.sent if t > 6.0 and i.ly != 0.0) >= rig.ctl.entered_at(TROTTING) + TROT_WAIT_S
    assert no_trot_while_lying(rig)
    assert_honest(rig)


def test_a_lost_command_is_sent_again():
    """2 通目の 2 が controller に届かなかった。伏せのまま RESEND_S（sim）たったら、伏せからもう一度 2 を送る。
    その間に来た BalanceStand は受け付け、窓口は立つに入った知らせまで 1001 のまま。"""
    rig = Rig(FakeController(lose_input=lambda n: n == 1))
    rig.run(8.0, api=[(0.0, STAND_UP), (3.0, BALANCE_STAND)])
    # 2.05 の 2 が落ち、RESEND_S（2.8）後の 4.85 に送り直す。立つの知らせは 4.90、4 はその 2.0 秒後
    assert rig.commands() == [(0.0, STAND_CMD), (2.05, STAND_CMD), (4.85, STAND_CMD), (6.9, TROT_CMD)]
    assert rig.ctl.state == TROTTING
    assert any("送り直します" in n for n in rig.b.notes)
    assert rig.code_at(4.85) == 1001 and rig.code_at(4.9) == 1013
    assert_honest(rig)


def test_a_lost_report_is_found_from_the_next_one():
    """「伏せに入った」の知らせが落ちた。橋渡しは脱力のつもりで 2 を送り直し、controller は伏せから立つへ行く。
    次の知らせ（伏せ → 立つ）で、そこから続ける。"""
    rig = Rig(FakeController(lose_report=lambda k: k == 0))
    rig.run(6.0, api=[(0.0, STAND_UP), (3.0, BALANCE_STAND)])
    assert rig.commands() == [(0.0, STAND_CMD), (2.8, STAND_CMD), (4.85, TROT_CMD)]
    assert [b for _, _, b in rig.ctl.switches] == [FIXED_DOWN, FIXED_STAND, TROTTING]
    assert any("送り直します" in n for n in rig.b.notes)
    assert any("fixed stand に切り替わりました（fixed down のはずでした）" in n for n in rig.b.notes)
    assert_honest(rig)


def test_the_bridge_gives_up_and_the_next_stand_up_starts_again():
    """controller に command が届かない（3 通続けて落ちる）。MAX_SENDS 回送ったらあきらめ、ノートに書いて送るのを
    やめる。窓口は確かめた状態のまま（脱力なので 1001）で、BalanceStand の 1013 も出さない。
    次の StandUp で、また初めから送る。"""
    assert MAX_SENDS == 3
    rig = Rig(FakeController(lose_input=lambda n: n < 3))
    rig.run(9.9, api=[(0.0, STAND_UP), (3.0, BALANCE_STAND)])
    assert rig.commands() == [(0.0, STAND_CMD), (2.8, STAND_CMD), (5.6, STAND_CMD)]
    assert sum("送り直します" in n for n in rig.b.notes) == 2
    assert any("passive から切り替わりません" in n and "Gazebo を起動し直して" in n for n in rig.b.notes)
    assert {c for _, c, _ in rig.trace} == {1001} and rig.ctl.state == PASSIVE
    rig.run(13.0, api=[(10.0, STAND_UP)])
    assert rig.commands()[3:] == [(10.0, STAND_CMD), (12.05, STAND_CMD)]
    assert rig.ctl.state == FIXED_STAND and rig.code_at(12.1) == 1002
    assert_honest(rig)


def test_stop_move_during_the_stand_holds_the_velocity_back():
    """立つ途中（歩くの知らせの前）に来た Move の速度は待たせ、StopMove で捨てる。立つのはそのまま続く。"""
    rig = Rig().run(6.0, api=[(0.0, STAND_UP), (3.0, BALANCE_STAND), (4.0, STOP_MOVE)],
                    move=(3.5, 3.9, '{"x":0.3,"y":0,"z":0}'))
    assert all((i.lx, i.ly, i.rx) == (0.0, 0.0, 0.0) for _, i, _ in rig.sent)
    assert rig.commands() == [(0.0, STAND_CMD), (2.05, STAND_CMD), (4.1, TROT_CMD)]
    assert rig.ctl.state == TROTTING and rig.code_at(3.9) == 1015 and rig.code_at(4.0) == 1013


def test_stand_down_before_the_trot_goes_straight_down():
    """BalanceStand の後、4 を送る前に StandDown：立つから 2 で伏せる。4 は送らない。controller が伏せに入ったと
    知らされたら、StandDown の 2 秒を待たずに 1004 を返す。"""
    rig = Rig().run(6.0, api=[(0.0, STAND_UP), (3.0, BALANCE_STAND), (3.6, STAND_DOWN)])
    assert rig.commands() == [(0.0, STAND_CMD), (2.05, STAND_CMD), (4.1, DOWN_CMD)]
    assert rig.ctl.state == FIXED_DOWN
    assert [rig.code_at(t) for t in (3.6, 4.1, 4.15, 6.0)] == [1013, 1013, 1004, 1004]
    assert not any("のままなので" in n for n in rig.b.notes)
    assert_honest(rig)


def test_stand_down_while_the_trot_is_on_the_way_waits_for_it():
    """4 を送って、歩くに入った知らせを待つ間に StandDown。知らせを待ってから、歩くから 2・2 で伏せる
    （知らせはどれも 0.2 秒遅れる）。"""
    rig = Rig(FakeController(report_delay=0.2))
    rig.run(8.0, api=[(0.0, STAND_UP), (3.0, BALANCE_STAND), (4.6, STAND_DOWN)])
    assert rig.commands() == [(0.0, STAND_CMD), (2.25, STAND_CMD), (4.5, TROT_CMD), (4.95, DOWN_CMD),
                              (7.2, DOWN_CMD)]
    assert [b for _, _, b in rig.ctl.switches] == [FIXED_DOWN, FIXED_STAND, TROTTING, FIXED_STAND, FIXED_DOWN]
    assert rig.b.error_code == 1004
    assert no_trot_while_lying(rig)
    assert_honest(rig, window=0.3)


def test_sit_and_rise_follow_the_reports():
    """おすわり（Gazebo では伏せ）から立ち上がる：伏せの知らせの後に 2、立つの知らせの後に 4。窓口の 1013 は
    立つに入った後。速度は、新しく歩くに入った知らせから TROT_WAIT_S 後（前の「歩く」の知らせは使わない）。"""
    rig = Rig().run(16.5, api=[(0.0, STAND_UP), (3.0, BALANCE_STAND), (8.0, SIT), (13.0, RISE_SIT)],
                    move=(15.5, 16.5, '{"x":0.3,"y":0,"z":0}'))
    assert rig.commands() == [(0.0, STAND_CMD), (2.05, STAND_CMD), (4.1, TROT_CMD), (8.0, DOWN_CMD),
                              (10.05, DOWN_CMD), (13.0, STAND_CMD), (15.05, TROT_CMD)]
    assert [rig.code_at(t) for t in (9.95, 10.0, 14.95, 15.0)] == [1013, 1007, 1007, 1013]
    assert rig.first_velocity() == pytest.approx(15.1 + TROT_WAIT_S + 0.05)
    assert no_trot_while_lying(rig)
    assert_honest(rig)


def test_a_fall_with_reports_stops_and_sends_nothing_more():
    """知らせ有りでも、転んだら（R38）速度 0 を 1 通送り、その後は何も送らない（起き上がらせない）。
    controller が後から切り替わっても、そこから送り直さない。"""
    rig = Rig().run(5.0, api=[(0.0, STAND_UP), (3.0, BALANCE_STAND)], move=(4.0, 5.0, '{"x":0.3,"y":0,"z":0}'))
    rig.b.observe(*UPRIGHT, 5.0)
    rig.b.observe(1.3, 0.0, 0.20, 5.05)
    assert rig.b.error_code == 1001 and fell(rig.b)
    i = rig.b.step(5.05)
    assert i.command == TROT_CMD and (i.lx, i.ly, i.rx) == (0.0, 0.0, 0.0)
    rig.b.controller_switched(FIXED_STAND, 5.1, from_state=TROTTING)
    assert all(rig.b.step(t) is None for t in (5.1, 7.5, 10.0)) and rig.b.error_code == 1001


def test_an_unknown_controller_state_is_left_through_fixed_stand():
    """橋渡しが使わない状態（balance test・free stand など）にいると知らされたら、2 で立つへ戻してから進む
    （どれも 2 で fixed stand へ行く）。"""
    b = BridgeCore(0.0, 0.0, reports_switches=True)
    b.handle(STAND_UP, "", 0.0, 0.0)
    assert b.step(0.0, 0.0).command == STAND_CMD
    b.controller_switched("balance test", 0.1, from_state=FIXED_STAND)
    assert b.step(0.25, 0.25) is None and b.step(0.3, 0.3).command == STAND_CMD   # 0.1 + SETTLE_S
    b.controller_switched(FIXED_STAND, 0.35, from_state="balance test")
    assert b.step(2.0, 2.0) is None and b.error_code == 1002


def test_controller_switch_lines():
    """controller のログ（UnitreeGuideController::update の "Switched from %s to %s"）。橋渡しのノードは /rosout の
    行をすべて parse_switch で読み、切り替えなら controller_switched に渡す（ノードは rclpy と Jazzy の型が要るので、
    ここでは読み方だけ確かめる）。"""
    assert parse_switch("Switched from passive to fixed down") == (PASSIVE, FIXED_DOWN)
    assert parse_switch("Switched from fixed down to fixed stand") == (FIXED_DOWN, FIXED_STAND)
    assert parse_switch("Switched from fixed stand to trotting") == (FIXED_STAND, TROTTING)
    assert parse_switch("Switched from trotting to fixed stand") == (TROTTING, FIXED_STAND)
    assert parse_switch("Switched from fixed stand to free stand") == (FIXED_STAND, "free stand")
    for text in ("Controller Manager Update Rate: 500 Hz", "受信: StandUp 1004", "Switched from passive", ""):
        assert parse_switch(text) is None
    node = (XACRO.parent / "gazebo_bridge.py").read_text(encoding="utf-8")
    assert "parse_switch(msg.msg)" in node and "controller_switched(" in node
