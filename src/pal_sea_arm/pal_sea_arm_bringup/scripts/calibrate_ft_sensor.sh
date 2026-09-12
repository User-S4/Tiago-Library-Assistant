#!/bin/bash

FT_SLAVE_TYPE="FTSensor"
ALL_FLAG=false

# Parse optional arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -a|--all) ALL_FLAG=true ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Function to get the positions of slaves of a specific type
get_slave_positions_from_type() {
    local slave_type=$1
    ethercat slaves | awk -v t="$slave_type" '$0 ~ t {print $1}'
}

# Function to check the calibration status of a specific slave
check_calibration_status() {
    local slave_position=$1
    local status
    while true; do
        status=$(ethercat upload -p "$slave_position" 0x6010 0 --type uint32 | awk '{print $2}')

        # Convert the status to binary and pad with leading zeros
        status_binary=$(echo "obase=2; $status" | bc)
        status_binary_32=$(printf "%032d" "$status_binary")

        # Extract the relevant bits from the status binary
        board_status_binary=${status_binary_32:30:2} # bits 0-1
        gauge_status_binary=${status_binary_32:24:6} # bits 2-7, one bit for each gauge
        reserved_status_binary=${status_binary_32:0:24} # bits 8-31

        # Convert board_status_binary to decimal
        board_status=$((2#$board_status_binary))
        if [ "$board_status" -eq 3 ]; then
            echo "Calibration ongoing for slave $slave_position..."
            sleep 1
        elif [ "$board_status" -eq 2 ]; then
            echo "Calibration complete with warning for slave $slave_position."
            break
        elif [ "$board_status" -eq 1 ]; then
            echo "Calibration complete with error for slave $slave_position."
            break
        elif [ "$board_status" -eq 0 ]; then
            echo "Calibration complete successfully for slave $slave_position."
            break
        else
            echo "Unknown board status for slave $slave_position."
            break
        fi
    done
}

# Function to calibrate a specific slave
calibrate_slave() {
    local slave_position=$1
    echo "Setting EtherCAT slave $slave_position to PREOP state..."
    ethercat states -p "$slave_position" PREOP
    sleep 0.1

    echo "Writing to register 0xFB01, subindex 1 on slave $slave_position..."
    ethercat download -p "$slave_position" 0xFB01 1 --type uint16 1
    sleep 0.1

    echo "Setting EtherCAT slave $slave_position to OP state..."
    ethercat states -p "$slave_position" OP

    echo "Waiting for calibration to complete on slave $slave_position..."
    check_calibration_status "$slave_position"

    echo "Calibration process for slave $slave_position completed successfully."
}

# Main script
SLAVE_POSITIONS=$(get_slave_positions_from_type "$FT_SLAVE_TYPE")

if [ -z "$SLAVE_POSITIONS" ]; then
    echo "No slaves of type '$FT_SLAVE_TYPE' found."
    exit 1
fi

echo "Found slaves of type '$FT_SLAVE_TYPE' at positions: $SLAVE_POSITIONS"

if [ "$ALL_FLAG" = true ]; then
    for SLAVE_POSITION in $SLAVE_POSITIONS; do
        calibrate_slave "$SLAVE_POSITION"
    done
else
    echo "Which slave do you want to calibrate?"
    select SLAVE_POSITION in $SLAVE_POSITIONS; do
        if [ -n "$SLAVE_POSITION" ]; then
            echo "You selected slave $SLAVE_POSITION for calibration."
            calibrate_slave "$SLAVE_POSITION"
            break
        else
            echo "Invalid selection. Please try again."
        fi
    done
fi
