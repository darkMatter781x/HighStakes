#!/bin/env bash
# Script for windows to automatically attach the v5 brain to wsl

# Check environment 
if [ ! -x "$(which usbipd.exe)" ]; then
  if [ ! -x "$(which powershell.exe)" ]; then
    echo "This script must be run in WSL. 
    If this is being run in wsl, make sure that windows executables are available" >&2
  else
    usbipd_install_link="https://learn.microsoft.com/en-us/windows/wsl/connect-usb#install-the-usbipd-win-project"
    echo "usbipd has not been installed. 
    Please see $usbipd_install_link" >&2
  fi
  exit 5
fi

if [ "$1" == 'bat' ]; then
  # Create bat file for this bash script
  if [ -p "$0" ]; then 
    # Script is likely being run in the .bat
    # In this state, the script cannot create a .bat, 
    # as it must be able to read the full file contents, which is not possible.
    echo "Cannot create bat when being run from bat!"
    exit 3
  fi
  bash_cmd="$(powershell.exe '$(Get-Command bash).Source' | sed 's/\r//')"
  # Creates encoded self to put inside the bat script
  encoded_self="$(<"$0" base64 -w 0)"
  # Eencoded self to put inside the bat script
  bat_encoded_self=
  echo '@echo off'
  echo "$bash_cmd -c "'"'"bash <(echo $encoded_self | base64 -d)"'"'""
  echo 'pause'
  exit 0
fi

# Get the bus-id of the v5
# Returns "null" when the device has been bound but is not currently connected
# Returns "" when the device has not been bound and is not currently connected 
get-v5() {
  usbipd.exe state \
    | jq -r '[ .Devices[] | select(.Description | test("VEX V5"; "i")).BusId | values ][0]'
}

frames=("⠋" "⠙" "⠹" "⠸" "⠼" "⠴" "⠦" "⠧" "⠇" "⠏")
i=0

stty_orig=$(stty -g)
trap "stty ${stty_orig}; echo -en \"\x1b[?25h\"" EXIT
stty -echo
echo -en "\x1b[?25l"

# Styling for spinner
bold_red="\x1b[1;91m"
gray="\x1b[1;30m"
left_side="$gray["
right_side="$gray]"
reset_style="\x1b[0m"

# Wait until brain is connected
while true; do
  bus_id="$(get-v5)";
  if [ "$bus_id" == "null" -o -z "$bus_id" ]; then
    core_spinner="$bold_red${frames[$((i++ % ${#frames[@]}))]}"
    full_spinner="$left_side$core_spinner$right_side$reset_style"
    printf "\rWaiting for brain to connect... $full_spinner"
  else
    echo -n ""
    echo "Brain is connected!"
    break
  fi
  sleep .25
done

# TODO: Investigate why this ignores sigint's when run in vscode 
# Attach brain to wsl
usbipd.exe attach -awb $bus_id