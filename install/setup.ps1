# generated from colcon_powershell/shell/template/prefix_chain.ps1.em

# This script extends the environment with the environment of other prefix
# paths which were sourced when this file was generated as well as all packages
# contained in this prefix path.

# function to source another script with conditional trace output
# first argument: the path of the script
function _colcon_prefix_chain_powershell_source_script {
  param (
    $_colcon_prefix_chain_powershell_source_script_param
  )
  # source script with conditional trace output
  if (Test-Path $_colcon_prefix_chain_powershell_source_script_param) {
    if ($env:COLCON_TRACE) {
      echo ". '$_colcon_prefix_chain_powershell_source_script_param'"
    }
    . "$_colcon_prefix_chain_powershell_source_script_param"
  } else {
    Write-Error "not found: '$_colcon_prefix_chain_powershell_source_script_param'"
  }
}

# source chained prefixes
_colcon_prefix_chain_powershell_source_script "/opt/ros/humble\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/fra501_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/carver_imu/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/carver_outdoor_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/ros2_humble/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/modelling_exercise_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/modelling_solution_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/simulation_noob_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/simulation_exercise_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/simulation_solution_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/FourTurtleSim_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/Carver_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/turtle_exam_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/turtle_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/mini_carver/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/carver_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/dev_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/megarover_samples_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/kaymarrr/FRA532_EX1_WS/install\local_setup.ps1"

# source this prefix
$env:COLCON_CURRENT_PREFIX=(Split-Path $PSCommandPath -Parent)
_colcon_prefix_chain_powershell_source_script "$env:COLCON_CURRENT_PREFIX\local_setup.ps1"
