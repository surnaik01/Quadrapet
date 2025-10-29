#!/bin/bash -e

load_env_if_present() {
  local env_file="$1"
  if [ -f "$env_file" ]; then
    set +x
    set -o allexport
    # shellcheck disable=SC1090
    source "$env_file"
    set +o allexport
    set -x
  fi
}

run_packer_container() {
  local args=("$@")
  if [ -n "${GITHUB_TOKEN:-}" ]; then
    set +x
    docker run --rm --privileged -v /dev:/dev -v "${PWD}":/build mkaczanowski/packer-builder-arm:latest "${args[@]}"
    set -x
  else
    docker run --rm --privileged -v /dev:/dev -v "${PWD}":/build mkaczanowski/packer-builder-arm:latest "${args[@]}"
  fi
}

cleanup_sanitized_env_file() {
  if [ -n "${SANITIZED_ENV_FILE:-}" ] && [ -f "${SANITIZED_ENV_FILE}" ]; then
    rm -f "${SANITIZED_ENV_FILE}"
  fi
}

set -x

# Get shortened git commit hash
GIT_COMMIT_SHORT=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")

#### Parse command line arguments
INCLUDE_KEYS=false
for arg in "$@"; do
  case $arg in
    --include-keys)
      INCLUDE_KEYS=true
      shift
      ;;
  esac
done

load_env_if_present ".env.local"

# Check if the full image exists (either as symlink or actual file)
if [ ! -e "pupOS_pios_full.img" ]; then
  echo "Full image not found. Running make_pios_full_image.sh..."
  ./make_pios_full_image.sh
else
  echo "Full image found. Skipping full image creation."
fi

#### Select which build to run
PACKER_ONLY="ai.arm.raspbian"
PACKER_BUILD_ARGS=()
if [ "$INCLUDE_KEYS" = true ]; then
  if [ ! -f ".env.local" ]; then
    echo "Error: --include-keys was passed but .env.local is missing in infra/quadrapet_image_builder/" >&2
    exit 1
  fi
  PACKER_ONLY="ai.arm.with-keys"
  echo "Including .env.local in image (ai_with_keys)."
  SANITIZED_ENV_FILE=$(mktemp .env.local.packer.XXXXXX)
  trap cleanup_sanitized_env_file EXIT
  sed -E '/^[[:space:]]*GITHUB_TOKEN\s*=.*/d' .env.local > "$SANITIZED_ENV_FILE"
  PACKER_BUILD_ARGS+=(-var "env_file_with_keys=${SANITIZED_ENV_FILE}")
fi

if [ -n "${GITHUB_TOKEN:-}" ]; then
  PACKER_BUILD_ARGS+=(-var "github_token=${GITHUB_TOKEN}")
fi

docker pull mkaczanowski/packer-builder-arm:latest
run_packer_container init pios_ai_arm64.pkr.hcl
run_packer_container build -only="$PACKER_ONLY" "${PACKER_BUILD_ARGS[@]}" pios_ai_arm64.pkr.hcl

# Rename the output image to include git commit hash
if [ -f "pupOS_pios_ai.img" ]; then
  DATE_SUFFIX=$(date +"%Y%m%d%H%M%S")
  if [ "$INCLUDE_KEYS" = true ]; then
    mv -f "pupOS_pios_ai.img" "pupOS_pios_ai_WITH_SECRETS_${GIT_COMMIT_SHORT}_${DATE_SUFFIX}.img"
    echo "Image saved as pupOS_pios_ai_WITH_SECRETS_${GIT_COMMIT_SHORT}_${DATE_SUFFIX}.img because --include-keys was used"
  else
    mv -f "pupOS_pios_ai.img" "pupOS_pios_ai_${GIT_COMMIT_SHORT}_${DATE_SUFFIX}.img"
    echo "Image saved as pupOS_pios_ai_${GIT_COMMIT_SHORT}_${DATE_SUFFIX}.img"
  fi
fi
