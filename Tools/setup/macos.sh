#! /usr/bin/env bash

set -euo pipefail

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Reinstall if --reinstall set
REINSTALL_FORMULAS=""
# Install simulation tools?
INSTALL_SIM=""

# Parse arguments
for arg in "$@"
do
	if [[ $arg == "--reinstall" ]]; then
		REINSTALL_FORMULAS=$arg
	elif [[ $arg == "--sim-tools" ]]; then
		INSTALL_SIM=$arg
	fi
done

if ! command -v brew &> /dev/null
then
	# install Homebrew if not installed yet
	/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"
fi

# Homebrew no longer resolves or trusts third-party tap dependencies
# implicitly. Install the actual PX4 toolchain packages directly instead of
# relying on the deprecated px4-dev meta-formula.
brew tap PX4/px4

if brew trust --help &> /dev/null; then
	brew trust PX4/px4
fi

PX4_BREW_PACKAGES=(
	ant
	astyle
	bash-completion
	ccache
	cmake
	fastdds
	flock
	genromfs
	kconfig-frontends
	ncurses
	ninja
	px4/px4/gcc-arm-none-eabi
)

if [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
	echo "Re-installing PX4 toolchain dependencies"
	brew doctor
	brew reinstall "${PX4_BREW_PACKAGES[@]}"
else
	echo "Installing PX4 toolchain dependencies"
	brew install "${PX4_BREW_PACKAGES[@]}"
fi

# Python dependencies
echo "Installing PX4 Python3 dependencies"
# We need to have future to install pymavlink later.
python3 -m pip install future
python3 -m pip install --user -r ${DIR}/requirements.txt

# Optional, but recommended additional simulation tools:
if [[ $INSTALL_SIM == "--sim-tools" ]]; then
	if brew ls --versions px4-sim > /dev/null; then
		brew install px4-sim
	elif [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
		brew reinstall px4-sim
	fi
fi

echo "All set! PX4 toolchain installed!"
