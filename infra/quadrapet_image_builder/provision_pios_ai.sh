#!/bin/bash -e

set -x

# Handle optional GitHub token for authenticated operations
GIT_ASKPASS_SCRIPT=""
GITHUB_TOKEN_CONFIGURED=false

cleanup_github_credentials() {
    if [ -n "${GIT_ASKPASS_SCRIPT:-}" ] && [ -f "${GIT_ASKPASS_SCRIPT}" ]; then
        rm -f "${GIT_ASKPASS_SCRIPT}"
    fi
    unset GIT_ASKPASS_SCRIPT
    unset GIT_ASKPASS
    unset GIT_TERMINAL_PROMPT
    unset GITHUB_TOKEN
    GITHUB_TOKEN_CONFIGURED=false
}

if [ -n "${GITHUB_TOKEN:-}" ]; then
    GITHUB_TOKEN_CONFIGURED=true
    GIT_ASKPASS_SCRIPT=$(mktemp /tmp/git-askpass-XXXXXX.sh)
    cat <<'EOF' > "${GIT_ASKPASS_SCRIPT}"
#!/bin/sh
case "$1" in
  Username*) echo "x-access-token" ;;
  Password*) echo "${GITHUB_TOKEN}" ;;
  *) echo "${GITHUB_TOKEN}" ;;
esac
EOF
    chmod 700 "${GIT_ASKPASS_SCRIPT}"
    export GIT_ASKPASS="${GIT_ASKPASS_SCRIPT}"
    export GIT_TERMINAL_PROMPT=0
    trap cleanup_github_credentials EXIT
fi

# Function to retry a command
retry_command() {
    local cmd="$1"
    local max_attempts=20
    local attempt=0

    until eval "$cmd" || [ $attempt -ge $max_attempts ]; do
        attempt=$((attempt + 1))
        echo "Attempt $attempt/$max_attempts failed. Retrying in 1 seconds..."
        sleep 1
    done

    if [ $attempt -ge $max_attempts ]; then
        echo "Command failed after $max_attempts attempts."
        return 1
    fi

    echo "Command succeeded!"
    return 0
}

############################## Basic setup ###############################################

export DEBIAN_FRONTEND=noninteractive

DEFAULT_USER=pi
mkdir -p /home/$DEFAULT_USER
chown -R $DEFAULT_USER:$DEFAULT_USER /home/$DEFAULT_USER

############################ Prepare monorepo ###############################################

# Prepare monorepo
cd /home/$DEFAULT_USER/quadrapetv3-monorepo/
git config --global --add safe.directory /home/$DEFAULT_USER/quadrapetv3-monorepo
git pull
git-lfs pull
chown -R $DEFAULT_USER:$DEFAULT_USER /home/$DEFAULT_USER/quadrapetv3-monorepo/

######################## Update and upgrade ########################
export APT_LISTCHANGES_FRONTEND=none
# (Optional) avoid services trying to start in chroot
printf '#!/bin/sh\nexit 101\n' > /usr/sbin/policy-rc.d && chmod +x /usr/sbin/policy-rc.d


apt-get update

apt-get -y \
  -o Dpkg::Options::="--force-confdef" \
  -o Dpkg::Options::="--force-confold" \
  upgrade

rm -f /usr/sbin/policy-rc.d

########################### Install rust ########################
export CARGO_HOME=/home/$DEFAULT_USER/.cargo
export RUSTUP_HOME=/home/$DEFAULT_USER/.rustup
# put these in bashrc as well
echo 'export CARGO_HOME=/home/pi/.cargo' >> /home/$DEFAULT_USER/.bashrc
echo 'export RUSTUP_HOME=/home/pi/.rustup' >> /home/$DEFAULT_USER/.bashrc
echo "source /home/$DEFAULT_USER/.cargo/env" >> /home/$DEFAULT_USER/.bashrc

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source /home/$DEFAULT_USER/.cargo/env

rustup target add aarch64-unknown-linux-gnu


############################### Install LLM deps ###########################
# Note: livekit 1.0.16 is incompatible with RPi OS due to glibc>=2.38 requirement
rm -f /usr/lib/python3.*/EXTERNALLY-MANAGED
pip install "livekit==1.0.13" "livekit-agents[cartesia,google,openai,deepgram,silero,turn-detector]==1.2.15"
pip install "python-dotenv"
pip install pandas

################################ HAILO DEPS ###########################
pip install "numpy<2" "opencv-python" "pyzmq"


############################### Build everything #############################################

# Checkout an older version of rosx_introspection to prevent GTest errors
cd /home/$DEFAULT_USER/quadrapetv3-monorepo/ros2_ws/src/common/rosx_introspection
git config --global --add safe.directory /home/pi/quadrapetv3-monorepo/ros2_ws/src/common/rosx_introspection
git checkout 3922e2c

# Build monorepo ros2 code
tmpfile=$(mktemp /tmp/ros2-build-output.XXXXXX)
if ! bash "/home/$DEFAULT_USER/quadrapetv3-monorepo/ros2_ws/build.sh" 2>&1 | tee "$tmpfile"; then
    echo "ros2 build script exited with non-zero status" >&2
    rm -f "$tmpfile"
    exit 1
fi

if grep -qi 'failed' "$tmpfile"; then
    echo "ros2 build script output contains 'failed'" >&2
    rm -f "$tmpfile"
    exit 1
fi

rm -f "$tmpfile"

# Build Rust GUI
cd /home/$DEFAULT_USER/quadrapetv3-monorepo/quadrapet-rs
cargo build --release --target aarch64-unknown-linux-gnu

# Install systemctl services
bash /home/$DEFAULT_USER/quadrapetv3-monorepo/quadrapet-rs/install_service.sh
bash /home/$DEFAULT_USER/quadrapetv3-monorepo/ai/llm-ui/agent-starter-python/install_service.sh
sudo systemctl enable systemd-time-wait-sync.service

if [ "$GITHUB_TOKEN_CONFIGURED" = true ]; then
    cleanup_github_credentials
    trap - EXIT
fi

# Try chowning again
chown -R $DEFAULT_USER:$DEFAULT_USER /home/$DEFAULT_USER
