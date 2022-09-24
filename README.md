# pico_lcd
---
## CMake 설정
cmake -S . -B build -G "Unix Makefiles" -DPICO_BOARD=pico

## CMake 빌드
cmake --build build -j4

## 다운로드
python tools\down.py

# 업데이트 내역
220818 lcd page 추가 확인 Cmake 환경 및 git 환경 추가
#
220819 LCD DEMO Page 완료 / 한글화 및 프레임 조정중 
#
220907 LCD 디자인 데모용 완료 
#
220924 메인 페이지 3개 기능페이지 12 이동테스트 완료
-버튼 숫자 BMP이미로 표현 테스트 

