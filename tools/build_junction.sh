#!/usr/bin/env bash
# 임의의 교차로를 OSM에서 받아 SUMO 시나리오(신호·보도·횡단보도·보행자·차량)로 만든다.
# 사용: SUMO_HOME=~/src/sumo-1.27.1-mesa tools/build_junction.sh <이름> <위도> <경도> [반경m=350]
# 차량 흐름은 SUMO randomTrips(무작위 출발·도착), 일반 차량은 Krauss/IDM/EIDM/ACC 4종 혼합.
set -euo pipefail
NAME=$1; LAT=$2; LON=$3; R=${4:-350}
cd "$(dirname "$0")/.."
export PATH="$SUMO_HOME/bin:$PATH"
OUT=env/$NAME; mkdir -p $OUT data/osm
read W S E N <<<$(python - <<PY
import math
lat,lon,r=$LAT,$LON,$R
dy=r/111320.0; dx=r/(111320.0*math.cos(math.radians(lat)))
print(f"{lon-dx:.5f} {lat-dy:.5f} {lon+dx:.5f} {lat+dy:.5f}")
PY
)
# Overpass 서버가 혼잡하면 거절하므로 대기 시간을 늘려 가며 재시도 (미러 2곳 번갈아)
ok=0
for wait in 0 15 30 60 90; do
  sleep $wait
  for H in https://overpass-api.de/api/interpreter https://overpass.kumi.systems/api/interpreter; do
    curl -s -m 120 -A "sumo-demo/1.0 (student project)" -o data/osm/$NAME.osm \
      --data-urlencode "data=[out:xml][timeout:90];(way[\"highway\"]($S,$W,$N,$E);>;);out meta;" "$H" || true
    if head -c 400 data/osm/$NAME.osm | grep -q "<osm "; then ok=1; echo "OSM 수신: $H"; break 2; fi
  done
done
[ $ok = 1 ] || { echo "OSM 다운로드 실패(서버 혼잡). 잠시 후 다시 실행하세요."; exit 1; }
OSM_STOPLINE_PROMOTE=1 OSM_ORIGIN="$LON,$LAT" OSM_BBOX="$W,$S,$E,$N" python tools/osm_to_sumo.py --osm data/osm/$NAME.osm --out $OUT
mv $OUT/cgc.nod.xml $OUT/net.nod.xml; mv $OUT/cgc.edg.xml $OUT/net.edg.xml
netconvert --node-files $OUT/net.nod.xml --edge-files $OUT/net.edg.xml -o $OUT/net.net.xml \
  --no-turnarounds true --junctions.join true --tls.join true --junctions.corner-detail 5 \
  --offset.disable-normalization true --sidewalks.guess true --sidewalks.guess.max-speed 14 \
  --crossings.guess true --crossings.guess.speed-threshold 14
# 일반 차량 4종 혼합 (청계천로 시나리오와 동일 정의 재사용)
python - <<PY
import re
r=open("env/cheonggyecheon/cgc.rou.xml").read()
m=re.search(r'    <vTypeDistribution id="carDist">.*?</vTypeDistribution>',r,re.S)
open("$OUT/vtypes.add.xml","w").write("<additional>\n"+m.group(0)+'\n    <vType id="ped" vClass="pedestrian" speed="1.3" color="0,0.4,1"/>\n</additional>\n')
PY
T=$SUMO_HOME/tools
python $T/randomTrips.py -n $OUT/net.net.xml -r $OUT/veh.rou.xml -o $OUT/veh.trips.xml -e 1800 -p ${VEH_PERIOD:-4} \
  --fringe-factor 40 --min-distance 200 --trip-attributes 'type="carDist" departLane="best" departSpeed="max"' --additional-files $OUT/vtypes.add.xml --validate
python $T/randomTrips.py -n $OUT/net.net.xml -r $OUT/ped.rou.xml -o $OUT/ped.trips.xml -e 1800 -p ${PED_PERIOD:-15} \
  --pedestrians --prefix ped --min-distance 80 --additional-files $OUT/vtypes.add.xml --validate
cat > $OUT/net.sumocfg <<CFG
<configuration><input><net-file value="net.net.xml"/>
<route-files value="veh.rou.xml,ped.rou.xml"/></input>
<gui_only><gui-settings-file value="net.gui.xml"/></gui_only></configuration>
CFG
cat > $OUT/net.gui.xml <<GUI
<viewsettings><scheme name="real world"/><viewport zoom="400" x="0" y="0" angle="0"/><delay value="80"/></viewsettings>
GUI
echo "완료: $OUT/net.sumocfg"
