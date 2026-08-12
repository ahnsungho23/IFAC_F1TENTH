#!/usr/bin/env bash
# =================================================================================================
# 2026_IFAC 로컬 Gitea 리모트 설정 — gitea를 기본 리모트로 (최초 1회, 멱등)
#
#   - 기존 origin(GitHub) -> github 로 이름 변경
#   - gitea -> origin (기본) 등록
#   - 모든 로컬 브랜치 upstream을 origin(gitea)으로 + remote.pushDefault=origin
#
# 사용법: ./setup_gitea.sh [리포 경로]     # 기본: ~/2026_IFAC
# =================================================================================================
set -euo pipefail
export GIT_SSH_COMMAND="ssh -o StrictHostKeyChecking=accept-new"

REPO="${1:-$HOME/2026_IFAC}"
REPO_SSH="ssh://parkm@10.1.1.100:2222/parkm/2026_IFAC.git"

[ -d "$REPO/.git" ] || { echo "리포를 찾을 수 없음: $REPO"; exit 1; }
cd "$REPO"

if git remote get-url origin 2>/dev/null | grep -q github.com; then
  git remote rename origin github
  echo "== origin(GitHub) -> github 로 이름 변경"
fi
if git remote get-url origin >/dev/null 2>&1; then
  git remote set-url origin "$REPO_SSH"
else
  git remote add origin "$REPO_SSH"
fi
echo "== origin = $REPO_SSH"

git fetch origin
for b in $(git branch --format='%(refname:short)'); do
  git show-ref --verify --quiet "refs/remotes/origin/$b" \
    && git branch --set-upstream-to="origin/$b" "$b" >/dev/null
done
git config remote.pushDefault origin

git ls-remote origin HEAD
echo "완료. git pull/push 기본 = gitea. GitHub는 'git push github <브랜치>'."
