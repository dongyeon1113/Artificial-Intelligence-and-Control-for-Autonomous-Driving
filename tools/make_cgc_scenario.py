"""청계천로 SUMO 시나리오 생성: 경로·교통 흐름·GUI 설정·sumocfg.

전제: tools/osm_to_sumo.py 로 만든 nod/edg + netconvert 로 만든 cgc.net.xml.

실제 상황 → 시뮬레이션 대응 (근거는 docs/cheonggyecheon_research.md)
  자율주행 셔틀(청계A01)         → 빨간 ego_type, 청계광장→광장시장 동행 경로, 시속 20km급
  끼어드는 차 (접촉 사고 원인)    → ① 측면 도로에서 본선으로 합류하는 차량
                                  ② 본선 2개 차로 사이를 오가는 차량(차선변경 허용)
  불법주정차·정차 차량            → 본선 우측 차로(0번)에 잠깐 서는 차량 → 뒤차가 옆 차로로 끼어듦
  오토바이                        → 폭이 좁고 공격적인 별도 차종
  일반 차량 controller 4종 혼합   → Krauss/IDM/EIDM/ACC (수업 기본 설정과 동일 비율)
"""
import argparse
import math
import os
import sys
import random

SUMO_HOME = os.environ.get("SUMO_HOME", os.path.expanduser("~/src/sumo-1.27.1-mesa"))
sys.path.append(os.path.join(SUMO_HOME, "tools"))
import sumolib  # noqa: E402

START_EDGE = "e73541403_1"      # 청계광장 (동행 첫 구간)
END_EDGE = "e1212424783_4"      # 청계5가·광장시장 부근 (동행 마지막 구간)


def shortest(net, a, b):
    path, _ = net.getShortestPath(net.getEdge(a), net.getEdge(b))
    return path


BAD_SIDE = {"e506246961_1b", "e329995230_2a", "e329995230_2b"}  # 합류부에서 계속 정체(순간이동) 확인


def side_sources(net, route_edges, min_len=25.0, limit=8):
    """본선 경로로 들어오는 측면 도로 중 충분히 긴 진입 엣지를 찾는다."""
    on = {e.getID() for e in route_edges}
    cand = []
    for e in route_edges:
        for inc in e.getFromNode().getIncoming():
            if inc.getID() in on or inc.getID() in BAD_SIDE or not inc.allows("passenger"):
                continue
            if inc.getLength() >= min_len and inc.getFromNode().getType() != "dead_end_x":
                cand.append(inc)
    seen, out = set(), []
    for e in cand:
        if e.getID() not in seen:
            seen.add(e.getID())
            out.append(e)
    return out[:limit]


def build(net_path, out_dir, args):
    net = sumolib.net.readNet(net_path)
    route = shortest(net, START_EDGE, END_EDGE)
    west = shortest(net, "e1212424785_1", "e350127794_6") if \
        net.hasEdge("e1212424785_1") and net.hasEdge("e350127794_6") else None
    srcs = side_sources(net, route)
    ids = " ".join(e.getID() for e in route)
    rng = random.Random(7)

    ctrl = [  # (id, model, prob, accel, decel, tau, minGap, maxSpeed, sigma, color)
        ("krauss", "Krauss", 0.35, 2.0, 4.5, 1.0, 2.5, 8.3, 0.3, "1,1,0"),
        ("idm", "IDM", 0.30, 1.8, 3.5, 1.2, 2.0, 8.3, None, "0,1,1"),
        ("eidm", "EIDM", 0.20, 2.0, 4.0, 1.1, 2.0, 8.3, None, "1,0.6,0"),
        ("acc", "ACC", 0.15, 2.0, 4.0, 1.5, 2.0, 8.3, None, "1,0,1"),
    ]
    L = ["<routes>"]
    L.append(f'    <vType id="ego_type" accel="1.5" decel="4.0" maxSpeed="{args.shuttle_kmh/3.6:.2f}" '
             f'length="5.0" width="2.0" color="1,0,0" vClass="passenger" guiShape="bus"/>')
    L.append('    <vTypeDistribution id="carDist">')
    for cid, model, p, acc, dec, tau, gap, vmax, sigma, color in ctrl:
        sg = f' sigma="{sigma}"' if sigma is not None else ""
        L.append(f'        <vType id="car_{cid}" probability="{p}" carFollowModel="{model}" accel="{acc}" '
                 f'decel="{dec}" tau="{tau}" minGap="{gap}" maxSpeed="{vmax}" speedDev="0.1" '
                 f'color="{color}" vClass="passenger" guiShape="passenger/sedan"{sg} '
                 f'lcStrategic="1" lcCooperative="0.5" lcAssertive="1.2" lcImpatience="0.2"/>')
    L.append("    </vTypeDistribution>")
    # 끼어드는 차: 차선변경에 적극적이고 간격이 좁은 차량 (택시·배달차 가정)
    L.append('    <vType id="cutin_car" accel="2.6" decel="4.5" tau="0.8" minGap="1.2" maxSpeed="9.7" '
             'speedDev="0.15" lcAssertive="3" lcImpatience="1" lcSpeedGain="3" lcStrategic="1" '
             'jmIgnoreFoeProb="0.35" jmTimegapMinor="0.3" jmIgnoreFoeSpeed="3" color="0.9,0.9,0.9" vClass="taxi" guiShape="passenger/sedan"/>')
    L.append('    <vType id="motorcycle" vClass="motorcycle" guiShape="motorcycle" length="2.2" width="0.8" '
             'accel="3.5" decel="5.0" maxSpeed="11" minGap="0.8" lcAssertive="3" lcImpatience="1" '
             'jmIgnoreFoeProb="0.2" color="0.2,0.2,0.2"/>')
    L.append('    <vType id="parked_car" accel="2.0" decel="4.5" maxSpeed="8.3" color="0.5,0.5,0.5" '
             'vClass="passenger" guiShape="passenger/van"/>')
    L.append(f'    <route id="main_east" edges="{ids}"/>')
    if west:
        L.append(f'    <route id="main_west" edges="{" ".join(e.getID() for e in west)}"/>')
    # 본선 통과 교통 (동행/서행)
    L.append(f'    <flow id="through_e" type="carDist" route="main_east" begin="0" end="100000" '
             f'vehsPerHour="{args.through_vph}" departLane="best" departSpeed="max"/>')
    if west:
        L.append(f'    <flow id="through_w" type="carDist" route="main_west" begin="0" end="100000" '
                 f'vehsPerHour="{args.through_vph}" departLane="best" departSpeed="max"/>')
    # 측면 도로 합류(끼어들기 원인 ①). 목적지는 본선 끝 엣지.
    for e in srcs:
        L.append(f'    <flow id="side_{e.getID()}" type="cutin_car" from="{e.getID()}" to="{END_EDGE}" '
                 f'begin="0" end="100000" vehsPerHour="{args.side_vph}" departLane="best" '
                 f'departSpeed="max"/>')
    # 오토바이: 본선 전체를 달리며 차선 사이를 파고든다
    L.append(f'    <flow id="moto" type="motorcycle" route="main_east" begin="0" end="100000" '
             f'vehsPerHour="{args.moto_vph}" departLane="best" departSpeed="max"/>')
    # 데모용 ego(셔틀): 학습 환경에서는 sumo_env가 TraCI로 직접 투입하므로 끌 수 있다.
    if args.demo_ego:
        L.append(f'    <flow id="ego_demo" type="ego_type" route="main_east" begin="{args.ego_begin}" '
                 f'end="100000" period="{args.ego_period}" departLane="0" departSpeed="0"/>')
    # 불법주정차: 본선 중간 엣지의 우측 차로(0번)에 잠깐 서는 차 (뒤차가 왼쪽으로 끼어들게 함)
    stop_edges = [e for e in route if e.getLength() > 60 and e.getLaneNumber() >= 2][:4]
    for i, e in enumerate(stop_edges):
        pos = e.getLength() * 0.6
        L.append(f'    <flow id="illegal_park_{i}" type="parked_car" route="main_east" begin="{args.ego_begin + 40 + 40 * i}" '
                 f'end="100000" period="{args.park_period}" departLane="0" departSpeed="max">'
                 f'<stop lane="{e.getID()}_0" endPos="{pos:.1f}" duration="{rng.randint(25, 50)}" parking="false"/>'
                 f'</flow>')
    L.append("</routes>")
    with open(os.path.join(out_dir, "cgc.rou.xml"), "w") as f:
        f.write("\n".join(L) + "\n")

    # GUI: 전체 도로가 한 화면에 보이도록 (x 0~2275, y -280~210)
    with open(os.path.join(out_dir, "cgc.gui.xml"), "w") as f:
        f.write('<viewsettings>\n    <scheme name="real world"/>\n'
                f'    <viewport zoom="{args.zoom}" x="{args.view_x}" y="{args.view_y}" angle="0"/>\n'
                '    <delay value="100"/>\n</viewsettings>\n')
    with open(os.path.join(out_dir, "cgc.sumocfg"), "w") as f:
        f.write('<configuration>\n    <input>\n        <net-file value="cgc.net.xml"/>\n'
                '        <route-files value="cgc.rou.xml"/>\n    </input>\n'
                '    <gui_only>\n        <gui-settings-file value="cgc.gui.xml"/>\n    </gui_only>\n'
                '</configuration>\n')
    print(f"경로 {len(route)}개 엣지, 측면 합류 {len(srcs)}곳, 불법주정차 {len(stop_edges)}곳")
    print("측면 합류 엣지:", [e.getID() for e in srcs])


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--net", default="env/cheonggyecheon/cgc.net.xml")
    ap.add_argument("--out", default="env/cheonggyecheon")
    ap.add_argument("--shuttle-kmh", type=float, default=20.0)
    ap.add_argument("--through-vph", type=int, default=500)
    ap.add_argument("--side-vph", type=int, default=40)
    ap.add_argument("--moto-vph", type=int, default=120)
    ap.add_argument("--park-period", type=int, default=90)
    ap.add_argument("--demo-ego", action="store_true", default=True)
    ap.add_argument("--ego-begin", type=int, default=60)  # 흐름은 출발시각 순으로 적어야 함
    ap.add_argument("--ego-period", type=int, default=300)
    ap.add_argument("--zoom", type=int, default=250)
    ap.add_argument("--view-x", type=float, default=1100)
    ap.add_argument("--view-y", type=float, default=-30)
    a = ap.parse_args()
    build(a.net, a.out, a)
