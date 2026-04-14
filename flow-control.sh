#!/bin/sh

###########################################
# FLOW CONTROL SCRIPT - HIGH PRECISION
# Compatible: busybox ash (TRB245)
# Requires: awk, ubus, modbus_client.rpc
#
# Precision features:
#  - Fast polling (0.3s) in throttle zone
#  - Predictive close: accounts for pipe
#    drain volume at current flow rate
#  - Per-target adaptive calibration
#  - Smooth quadratic valve curve from 80%
###########################################

###########################################
# CONFIGURATION
###########################################

TARGET_LITERS=10.0       # Target volume in liters
CHECK_INTERVAL=1         # Polling interval before throttle zone (seconds)
FAST_INTERVAL=300000     # Polling interval IN throttle zone (microseconds = 0.3s)
RS485_MIN_GAP=0.25       # Reduced gap in throttle zone for faster response

# Smooth throttle curve
THROTTLE_START_PCT=80    # % of target filled → begin closing valve
VALVE_FULL_OPEN=9000     # Fully open position
VALVE_MIN_TRICKLE=1800   # Minimum trickle (raise if flow stalls)
CURVE_POWER=2.0          # 1.0=linear, 2.0=quadratic, 3.0=cubic

# Predictive close:
# Valve closes when: remaining <= CLOSE_OFFSET_L + (flow_rate * PIPE_DRAIN_TIME)
# PIPE_DRAIN_TIME = seconds of flow still coming after valve closes
# Measure this once: close valve, count how long water keeps flowing
PIPE_DRAIN_TIME=2.0      # seconds of residual flow after valve close

# Base close offset (calibrated per target)
CLOSE_OFFSET_L=0.50

# Adaptive calibration
CALIB_DIR="/var/run"
CALIB_LEARNING_RATE=0.50  # 50% correction per run — converges in 3 runs
CALIB_MAX_ADJUST=0.30     # Max shift per run in liters
CALIB_MIN_OFFSET=0.00
CALIB_MAX_OFFSET=2.50
CALIB_DEADZONE=0.015      # Ignore errors under 15ml

# Stall recovery
STALL_BUMP=200
STALL_MAX_POS=4000
STALL_AFTER=3
STALL_TIMEOUT=90

LOCKFILE="/var/run/flow_control.lock"

###########################################
# GLOBALS
###########################################

LAST_RS485_CALL=0
CURRENT_VALVE_POS=-1
THROTTLE_START_TIME=0
IN_THROTTLE=0
ZERO_RATE_COUNT=0
CURRENT_TRICKLE_MIN=$VALVE_MIN_TRICKLE

###########################################
# LOGGING
###########################################

log() { echo "[$(date '+%H:%M:%S')] $*"; }
die() { log "ERROR: $*"; exit 1; }

###########################################
# CLEANUP
###########################################

cleanup() { rm -f "$LOCKFILE"; }
trap cleanup EXIT INT TERM

###########################################
# SINGLE INSTANCE LOCK
###########################################

if [ -f "$LOCKFILE" ]; then
    OLD_PID=$(cat "$LOCKFILE")
    if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
        die "Script already running (PID $OLD_PID)"
    else
        log "Removing stale lock file"
        rm -f "$LOCKFILE"
    fi
fi
echo $$ > "$LOCKFILE"

###########################################
# CALIBRATION (per target volume)
###########################################

calib_file() {
    safe=$(echo "$TARGET_LITERS" | tr '.' '_')
    echo "${CALIB_DIR}/flow_calib_${safe}L.dat"
}

load_calibration() {
    CFILE=$(calib_file)
    if [ -f "$CFILE" ]; then
        saved=$(grep '^CLOSE_OFFSET_L=' "$CFILE" | cut -d'=' -f2 | tr -d ' ')
        if echo "$saved" | grep -qE '^[0-9]+(\.[0-9]+)?$'; then
            CLOSE_OFFSET_L="$saved"
            log "Calibration loaded for ${TARGET_LITERS}L: base offset=${CLOSE_OFFSET_L}L"
        else
            log "Calibration invalid, using default ${CLOSE_OFFSET_L}L"
        fi
    else
        log "No calibration for ${TARGET_LITERS}L yet, using default ${CLOSE_OFFSET_L}L"
    fi
}

save_calibration() {
    CFILE=$(calib_file)
    echo "CLOSE_OFFSET_L=$CLOSE_OFFSET_L" > "$CFILE"
}

update_calibration() {
    overshoot_liters="$1"

    significant=$(awk "BEGIN {
        v = $overshoot_liters; if (v < 0) v = -v
        print (v > $CALIB_DEADZONE) ? 1 : 0
    }")
    if [ "$significant" -eq 0 ]; then
        log "Calibration: error=${overshoot_liters}L within ±${CALIB_DEADZONE}L — no adjustment"
        return
    fi

    new_offset=$(awk "BEGIN {
        adj = $CALIB_LEARNING_RATE * $overshoot_liters
        if (adj >  $CALIB_MAX_ADJUST) adj =  $CALIB_MAX_ADJUST
        if (adj < -$CALIB_MAX_ADJUST) adj = -$CALIB_MAX_ADJUST
        new = $CLOSE_OFFSET_L + adj
        if (new < $CALIB_MIN_OFFSET) new = $CALIB_MIN_OFFSET
        if (new > $CALIB_MAX_OFFSET) new = $CALIB_MAX_OFFSET
        printf \"%.4f\", new
    }")

    log "Calibration [${TARGET_LITERS}L]: error=${overshoot_liters}L → offset: ${CLOSE_OFFSET_L}L → ${new_offset}L"
    CLOSE_OFFSET_L="$new_offset"
    save_calibration
}

###########################################
# RS485 BUS GUARD
###########################################

rs485_guard() {
    now=$(date +%s.%N)
    if [ "$LAST_RS485_CALL" != "0" ]; then
        need_wait=$(awk "BEGIN {
            elapsed = $now - $LAST_RS485_CALL
            print (elapsed < $RS485_MIN_GAP) ? 1 : 0
        }")
        if [ "$need_wait" -eq 1 ]; then
            wait_us=$(awk "BEGIN {
                elapsed = $now - $LAST_RS485_CALL
                us = int(($RS485_MIN_GAP - elapsed) * 1000000)
                print (us > 0) ? us : 1
            }")
            usleep "$wait_us"
        fi
    fi
    LAST_RS485_CALL=$(date +%s.%N)
}

###########################################
# MODBUS HELPER
###########################################

modbus_call() {
    FUNC="$1"; FREG="$2"; RCNT="$3"; MID="$4"
    rs485_guard
    ubus call modbus_client.rpc serial.test "{
        \"id\":$MID,
        \"timeout\":2,
        \"function\":$FUNC,
        \"first_reg\":$FREG,
        \"reg_count\":\"$RCNT\",
        \"data_type\":\"16bit_int_hi_first\",
        \"no_brackets\":0,
        \"serial_type\":\"/dev/rs485\",
        \"baudrate\":9600,
        \"databits\":8,
        \"stopbits\":1,
        \"parity\":\"none\",
        \"flowcontrol\":\"none\"
    }" 2>/dev/null
}

###########################################
# FLOW METER READ
###########################################

read_flow_meter() {
    response=$(modbus_call 3 3 4 1)

    result=$(echo "$response" | grep -o '"result":"[^"]*"' | sed 's/"result":"//;s/"//')
    if [ -z "$result" ]; then
        result=$(echo "$response" | grep -o '"result": *"[^"]*"' | sed 's/"result": *"//;s/"//')
    fi
    result=$(echo "$result" | tr -d '[]')

    if [ -z "$result" ]; then
        log "WARNING: Could not parse meter response: $response"
        echo "0.000"
        return
    fi

    T1=$(echo "$result" | cut -d',' -f1 | tr -d ' ')
    T2=$(echo "$result" | cut -d',' -f2 | tr -d ' ')
    T3=$(echo "$result" | cut -d',' -f3 | tr -d ' ')
    T4=$(echo "$result" | cut -d',' -f4 | tr -d ' ')

    for val in "$T1" "$T2" "$T3" "$T4"; do
        if ! echo "$val" | grep -qE '^[0-9]+$'; then
            log "WARNING: Non-numeric value: '$val' in '$result'"
            echo "0.000"
            return
        fi
    done

    awk "BEGIN { printf \"%.3f\", $T1 * 100000000 + $T2 * 10000 + $T3 + $T4 / 1000 }"
}

###########################################
# VALVE POSITION
###########################################

set_valve_position() {
    POS="$1"
    if [ "$POS" = "$CURRENT_VALVE_POS" ]; then
        return
    fi
    modbus_call 16 2 "$POS" 2 >/dev/null
    CURRENT_VALVE_POS="$POS"
}

###########################################
# SMOOTH VALVE CURVE
###########################################

get_smooth_valve_pos() {
    remaining="$1"
    effective_close="$2"   # CLOSE_OFFSET_L + predictive drain volume

    throttle_start_rem=$(awk "BEGIN {
        printf \"%.4f\", $TARGET_LITERS * (1 - $THROTTLE_START_PCT / 100)
    }")

    # Full open zone
    in_throttle=$(awk "BEGIN { print ($remaining <= $throttle_start_rem) ? 1 : 0 }")
    if [ "$in_throttle" -eq 0 ]; then
        echo "$VALVE_FULL_OPEN"
        return
    fi

    # Close zone
    should_close=$(awk "BEGIN { print ($remaining <= $effective_close) ? 1 : 0 }")
    if [ "$should_close" -eq 1 ]; then
        echo "0"
        return
    fi

    # Smooth curve zone
    pos=$(awk "BEGIN {
        range = $throttle_start_rem - $effective_close
        if (range <= 0) { print $CURRENT_TRICKLE_MIN; exit }
        progress = ($throttle_start_rem - $remaining) / range
        if (progress < 0) progress = 0
        if (progress > 1) progress = 1
        curved = progress ^ $CURVE_POWER
        valve  = $VALVE_FULL_OPEN - ($VALVE_FULL_OPEN - $CURRENT_TRICKLE_MIN) * curved
        if (valve < $CURRENT_TRICKLE_MIN) valve = $CURRENT_TRICKLE_MIN
        if (valve > $VALVE_FULL_OPEN)     valve = $VALVE_FULL_OPEN
        printf \"%d\", valve
    }")
    echo "$pos"
}

###########################################
# RESET FLOW METER
###########################################

reset_flow_meter() {
    log "Resetting flow meter..."
    modbus_call 6 3 0 1 >/dev/null
}

###########################################
# WAIT FOR FLOW TO START
###########################################

wait_for_flow() {
    log "Waiting for flow to start..."
    prev=$(read_flow_meter)
    attempts=0
    while true; do
        sleep "$CHECK_INTERVAL"
        curr=$(read_flow_meter)
        attempts=$((attempts + 1))
        is_flowing=$(awk "BEGIN { print ($curr > $prev) ? 1 : 0 }")
        if [ "$is_flowing" -eq 1 ]; then
            log "Flow detected: $curr L (was $prev L)"
            prev_flow=$curr
            prev_time=$(date +%s.%N)
            return 0
        fi
        mod=$(awk "BEGIN { print ($attempts % 5 == 0) ? 1 : 0 }")
        [ "$mod" -eq 1 ] && log "Still waiting... meter=$curr L"
        prev=$curr
    done
}

###########################################
# MAIN
###########################################

load_calibration

throttle_start_L=$(awk "BEGIN { printf \"%.2f\", $TARGET_LITERS * $THROTTLE_START_PCT / 100 }")

echo "======================================="
echo " Flow Control  (PID $$)"
echo " Target    : ${TARGET_LITERS} L"
echo " Throttle  : smooth curve from ${throttle_start_L}L (${THROTTLE_START_PCT}%)"
echo " Curve     : power=${CURVE_POWER} | ${VALVE_FULL_OPEN} → ${VALVE_MIN_TRICKLE}"
echo " Close at  : ${CLOSE_OFFSET_L}L base + predictive (${PIPE_DRAIN_TIME}s drain)"
echo " Polling   : ${CHECK_INTERVAL}s normal / 0.3s in throttle zone"
echo " Stall     : bump +${STALL_BUMP} after ${STALL_AFTER}s, max=${STALL_MAX_POS}"
echo "======================================="

log "RS485 bus settling..."
sleep 2

log "Opening valve fully..."
set_valve_position $VALVE_FULL_OPEN
sleep 1

wait_for_flow

log "Monitoring..."

flow_rate="0.0000"

while true; do

    # Use fast polling in throttle zone, normal polling before it
    if [ "$IN_THROTTLE" -eq 1 ]; then
        usleep "$FAST_INTERVAL"
    else
        sleep "$CHECK_INTERVAL"
    fi

    curr_flow=$(read_flow_meter)
    curr_time=$(date +%s.%N)

    delta_vol=$(awk "BEGIN { print $curr_flow - $prev_flow }")
    delta_t=$(awk "BEGIN { print $curr_time - $prev_time }")

    # Update flow rate only when delta_t is meaningful
    new_rate=$(awk "BEGIN {
        if ($delta_t > 0.1)
            printf \"%.4f\", $delta_vol / $delta_t
        else
            print \"$flow_rate\"
    }")
    flow_rate="$new_rate"

    filled_pct=$(awk "BEGIN { printf \"%.1f\", ($curr_flow / $TARGET_LITERS) * 100 }")
    remaining=$(awk "BEGIN { printf \"%.3f\", $TARGET_LITERS - $curr_flow }")

    # Predictive close threshold:
    # = base calibrated offset + volume that will drain during pipe drain time
    # This adapts automatically to both fast and slow flow rates
    effective_close=$(awk "BEGIN {
        drain_vol = $flow_rate * $PIPE_DRAIN_TIME
        total = $CLOSE_OFFSET_L + drain_vol
        if (total < $CLOSE_OFFSET_L) total = $CLOSE_OFFSET_L
        printf \"%.4f\", total
    }")

    target_pos=$(get_smooth_valve_pos "$remaining" "$effective_close")

    # Throttle zone entry
    throttle_start_rem=$(awk "BEGIN {
        printf \"%.4f\", $TARGET_LITERS * (1 - $THROTTLE_START_PCT / 100)
    }")
    entering=$(awk "BEGIN {
        print ($remaining <= $throttle_start_rem && $IN_THROTTLE == 0) ? 1 : 0
    }")
    if [ "$entering" -eq 1 ]; then
        IN_THROTTLE=1
        THROTTLE_START_TIME=$(date +%s)
        log "Throttle active — ${curr_flow}L filled, ${remaining}L rem, rate=${flow_rate}L/s"
        log "Predictive close: base=${CLOSE_OFFSET_L}L + drain=${PIPE_DRAIN_TIME}s × ${flow_rate}L/s = ${effective_close}L"
    fi

    # Stall detection & recovery (throttle zone only)
    if [ "$IN_THROTTLE" -eq 1 ] && [ "$target_pos" -ne 0 ]; then

        now_s=$(date +%s)
        elapsed=$((now_s - THROTTLE_START_TIME))
        if [ "$elapsed" -ge "$STALL_TIMEOUT" ]; then
            log "TIMEOUT: ${elapsed}s in throttle, closing. Meter: $curr_flow L"
            set_valve_position 0
            CURRENT_VALVE_POS=0
            break
        fi

        is_zero=$(awk "BEGIN { print ($flow_rate <= 0.001) ? 1 : 0 }")
        if [ "$is_zero" -eq 1 ]; then
            ZERO_RATE_COUNT=$((ZERO_RATE_COUNT + 1))
            if [ "$ZERO_RATE_COUNT" -ge "$STALL_AFTER" ]; then
                new_min=$(awk "BEGIN {
                    p = $CURRENT_TRICKLE_MIN + $STALL_BUMP
                    print (p > $STALL_MAX_POS) ? $STALL_MAX_POS : p
                }")
                if [ "$new_min" != "$CURRENT_TRICKLE_MIN" ]; then
                    log "STALL: bumping trickle floor $CURRENT_TRICKLE_MIN → $new_min"
                    CURRENT_TRICKLE_MIN=$new_min
                    ZERO_RATE_COUNT=0
                    target_pos=$(get_smooth_valve_pos "$remaining" "$effective_close")
                else
                    log "STALL: at max trickle floor ($STALL_MAX_POS)"
                fi
            fi
        else
            ZERO_RATE_COUNT=0
        fi
    fi

    printf "[%s] %6.3f / %s L | %5.1f%% | rem: %s L | %.4f L/s | close@: %sL | valve: %s\n" \
        "$(date '+%H:%M:%S')" \
        "$curr_flow" "$TARGET_LITERS" "$filled_pct" "$remaining" \
        "$flow_rate" "$effective_close" "$target_pos"

    set_valve_position "$target_pos"

    if [ "$target_pos" -eq 0 ]; then
        log "Valve closed. Meter: $curr_flow L | rate was: ${flow_rate}L/s"
        break
    fi

    prev_flow=$curr_flow
    prev_time=$curr_time
done

# Wait for pipe to fully drain then read final
sleep_drain=$(awk "BEGIN { printf \"%d\", $PIPE_DRAIN_TIME + 2 }")
sleep "$sleep_drain"
final_flow=$(read_flow_meter)
overshoot=$(awk "BEGIN { printf \"%.3f\", $final_flow - $TARGET_LITERS }")

log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
log "  Target : ${TARGET_LITERS} L"
log "  Final  : ${final_flow} L"
log "  Error  : ${overshoot} L"
log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

update_calibration "$overshoot"

reset_flow_meter

echo "======================================="
echo " Done."
echo "======================================="
exit 0