# 좌우 이동 오토마타

3 V 마이크로 DC 모터 → 80:1 3단 평기어 → 슬라이더-크랭크 → 양철 로봇이 뚜껑 위를 좌우로 오가는 오토마타.
부품은 Blender로 모델링해 GLB로 내보냈고, 웹에서 three.js로 실제 기구학대로 구동합니다.

## 실행 (웹)

```bash
python3 -m http.server 8000      # 저장소 루트에서
# 브라우저: http://localhost:8000/web/
```

`web/index.html` 은 `../assets/parts/*.glb`, `../spec/kinematics.json` 을 상대경로로 불러오므로 **저장소 루트**를 서빙해야 합니다 (GitHub Pages도 동일). `file://` 로는 열리지 않습니다.

## 구성

| 경로 | 내용 |
| --- | --- |
| `assets/parts/` | 부품 GLB 5개: `drive`(모터·기어열·섀시), `slide`(크랭크·로드·캐리지·가이드), `figure`(로봇), `electronics`(배터리·스위치·LED·배선), `housing`(나무 상자 + 아크릴 앞판) |
| `spec/kinematics.json` | 모든 부품이 공유하는 계약: 노드 원점, 기어 잇수·방향, 크랭크/로드/캐리지 수치 (좌표 Z-up, mm) |
| `tools/kinematics.py`, `web/kinematics.js` | 같은 기구학 수식의 Python / JS 구현 |
| `tools/check_assembly.py` | 0~360° 전 구간 조립 간섭 검사 (trimesh + fcl) |
| `web/` | 실시간 3D 뷰어 (스위치, 속도, 분해도, 내부 보기, 기구학 그래프, 렌더 갤러리) |
| `render/`, `web/renders/` | Blender(Cycles, bpy) 렌더 스크립트와 결과물 |

## 기구

- 모터 → P1(10T) → G1(40T) · P2(10T) → G2(40T) · P3(12T) → G3(60T): 4 × 4 × 5 = **80:1** (모듈 1)
- 크랭크 반지름 26 mm, 커넥팅 로드 100 mm, 캐리지 핀 높이 z = 92 mm
- 캐리지 행정 **56.6 mm** (x = −28.3 … +28.3), 크랭크 한 바퀴에 한 번 왕복. 한쪽이 눌리는 이유는 로드가 기울어 있기 때문입니다.
- 3 V 모터 무부하 약 6000 rpm → 크랭크 약 75 rpm. 웹의 기본 속도는 기어 맞물림이 보이도록 12 rpm 입니다.

## 조립 간섭 검사

```bash
pip install trimesh python-fcl networkx scipy rtree numpy
python tools/check_assembly.py --step 10
```

결과(10~15° 간격): 기어 3단 맞물림 쌍은 전 구간에서 충돌이 없고, 각도와 무관한 접촉 17건은 삽입·체결부(축/핀/솔더/스위치 등)입니다.
**각도 의존 접촉 1건**: 크랭크 핀 어셈블리의 뒷면이 θ≈315~345°에서 G2·P3 기어 앞면과 맞닿습니다 (여유 0 mm). 실물이라면 크랭크와 기어 사이에 0.5 mm 와셔(스페이서)를 넣거나 G3 축을 앞으로 0.5 mm 빼는 것을 권장합니다.
