#!/bin/bash

# Define the usage function
usage() {
  echo "Usage: $0 [-h] [-c <config>] [-s <system>] [-f <floating_license_platform>]" 1>&2
  echo "Setup the solver interface files and creation of the solver" 1>&2
  echo "" 1>&2
  echo "Required options:" 1>&2
  echo "  -c, --config <config>                 One or more configuration files" 1>&2
  echo "  -s, --system <system>                 System argument" 1>&2
  echo "" 1>&2
  echo "Optional options:" 1>&2
  echo "  -h, --help                            Show this help message and exit" 1>&2
  echo "  -f, --floating_license_platform       Floating license platform argument with the available options X86 or ARM" 1>&2
  echo "  -n, --no-build                        Do not build the solver, only the interface" 1>&2
  echo "" 1>&2
  echo "Examples of usage:" 1>&2
  echo "  $0 -c config1.yaml -c config2.yaml -s system1 -f X86" 1>&2
  echo "  $0 -c \"config1.yaml config2.yaml\" -s system1 -f ARM" 1>&2
  exit 1
}

activate_venv() {
  # check if venv exist, if not then create venv
  #
  VENV_DIR="./venv/"
  if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv venv
    source venv/bin/activate
    pip3 install casadi==3.5.1 \
      numpy==1.18.3 \
      pyyaml \
      requests==2.26.0 \
      scipy==1.4.1 \
      suds
    echo -e "\n Done with setting op the virtual environment"
  fi

  source venv/bin/activate
  echo -e "\n Virtual environment is activated \n"
}

# Initialize the variables
config=()
system=""
floating_license_platform=""
build=true

# Parse the command line arguments
while getopts ":hnc:s:f:b:" o; do
  case "${o}" in
  h)
    usage
    ;;
  c)
    config+=(${OPTARG})
    ;;
  s)
    system=${OPTARG}
    ;;
  f)
    floating_license_platform=${OPTARG}
    ;;
  n)
    build=false
    ;;
  *)
    usage
    ;;
  esac
done
shift $((OPTIND - 1))

# Check if the help option is invoked
if [[ "${1}" == "-h" || "${1}" == "--help" ]]; then
  usage
fi

# Check if the required arguments are provided
if [[ -z "${system}" || -z "${build}" ]]; then
  echo "Error: Missing required arguments" >&2
  usage
fi

# Check if the optional arguments are provided
if [[ "${#config[@]}" -eq 0 ]]; then
  echo "Warning: No configuration files provided" >&2
fi

activate_venv

# Launch the Python script with the arguments
python3 scripts/include/setup_script.py -c "${config[@]}" -s "${system}" -f "${floating_license_platform}" -b "${build}"

deactivate
