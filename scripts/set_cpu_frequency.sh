#!/bin/bash

# Check if the script is run as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root"
    exit 1
fi

# Function to list available frequencies
list_available_frequencies() {
    local cpu0_path="/sys/devices/system/cpu/cpu0/cpufreq"
    if [ -f "$cpu0_path/scaling_available_frequencies" ]; then
        cat "$cpu0_path/scaling_available_frequencies"
    else
        echo "No available frequencies found."
        exit 1
    fi
}

# Function to set CPU frequency for all cores
set_cpu_frequency() {
    local frequency=$1
    
    for cpu in /sys/devices/system/cpu/cpu[0-7]; do
        echo "Setting CPU frequency for $(basename $cpu)..."
        
        # Set the governor to "userspace" to manually control the frequency
        echo "userspace" > $cpu/cpufreq/scaling_governor
        
        # Set the desired frequency
        echo $frequency > $cpu/cpufreq/scaling_setspeed
    done
}

# List available frequencies and prompt the user to choose one
echo "Available frequencies (in kHz):"
available_frequencies=$(list_available_frequencies)
echo $available_frequencies | tr ' ' '\n'

echo ""
read -p "Enter the frequency you want to set (in kHz): " selected_frequency

# Check if the selected frequency is valid
if [[ ! " $available_frequencies " =~ " $selected_frequency " ]]; then
    echo "Error: Invalid frequency selected."
    exit 1
fi

# Set the CPU frequency
set_cpu_frequency $selected_frequency

# Verify the setting
for cpu in /sys/devices/system/cpu/cpu[0-7]; do
    current_freq=$(cat $cpu/cpufreq/scaling_cur_freq)
    echo "$(basename $cpu) is now running at $current_freq kHz"
done

