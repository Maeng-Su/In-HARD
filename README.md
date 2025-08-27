# In-HARD
In-HARD 데이터셋, Pick&amp;Place, RoboDK

# 폴더 구조
- Online: In-HARD 데이터셋 (용량이 큼으로 직접 다운로드해야함.)
- maeng: 맹성찬 작업 폴더
- jhkim: 김지희 작업 폴더

# 자주쓰는 명령어
### 100메가 이하 파일만 add
- find . -type f -size -100M -not -path "./.git/*" | xargs git add