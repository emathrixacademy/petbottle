#!/bin/bash
#############################################
# Raspberry Pi fallback AP setup
# Creates an always-available WiFi hotspot
# that activates whenever no known client
# network is connected.
#
# SSID:     PiRobot
# Password: robotpi123
# Pi IP:    192.168.50.1
#############################################

set -e

AP_CONN="pi-fallback-ap"
AP_SSID="PiRobot"
AP_PASS="robotpi123"
AP_IFACE="wlan0"
AP_IP="192.168.50.1/24"
WATCHDOG="/usr/local/bin/pi-fallback-ap-watchdog.sh"
SERVICE="/etc/systemd/system/pi-fallback-ap.service"
TIMER="/etc/systemd/system/pi-fallback-ap.timer"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
log()  { echo -e "${GREEN}[INFO]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
err()  { echo -e "${RED}[ERR ]${NC} $1" >&2; }

[ "$EUID" -eq 0 ] || { err "Run with sudo: sudo bash $0"; exit 1; }

# ── 1. AP profile ─────────────────────────────────────────────
if nmcli -t -f NAME con show | grep -qx "$AP_CONN"; then
    log "AP profile '$AP_CONN' already exists — updating settings"
    nmcli con modify "$AP_CONN" \
        802-11-wireless.ssid "$AP_SSID" \
        802-11-wireless.mode ap \
        802-11-wireless.band bg \
        ipv4.method shared \
        ipv4.addresses "$AP_IP" \
        ipv6.method disabled \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "$AP_PASS" \
        connection.autoconnect no
else
    log "Creating AP profile '$AP_CONN'"
    nmcli con add type wifi ifname "$AP_IFACE" con-name "$AP_CONN" ssid "$AP_SSID"
    nmcli con modify "$AP_CONN" \
        802-11-wireless.mode ap \
        802-11-wireless.band bg \
        ipv4.method shared \
        ipv4.addresses "$AP_IP" \
        ipv6.method disabled \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "$AP_PASS" \
        connection.autoconnect no
fi

# ── 2. Watchdog script ────────────────────────────────────────
log "Installing watchdog at $WATCHDOG"
cat > "$WATCHDOG" <<WDEOF
#!/bin/bash
# Keeps the fallback AP up whenever no client WiFi is connected.
AP_CONN="pi-fallback-ap"
IFACE="wlan0"

ap_active() {
    nmcli -t -f NAME con show --active | grep -qx "\$AP_CONN"
}

client_active() {
    nmcli -t -f NAME,TYPE,DEVICE con show --active 2>/dev/null | \
        awk -F: -v ap="\$AP_CONN" -v i="\$IFACE" \
            '\$2=="802-11-wireless" && \$3==i && \$1!=ap {f=1} END {exit !f}'
}

scan_for_known() {
    nmcli device wifi rescan ifname "\$IFACE" 2>/dev/null || true
    sleep 3
    local visible known ssid
    visible=\$(nmcli -t -f SSID dev wifi list ifname "\$IFACE" 2>/dev/null | sort -u)
    known=\$(nmcli -t -f NAME,TYPE con show | awk -F: '\$2=="802-11-wireless"{print \$1}' | grep -vx "\$AP_CONN")
    while IFS= read -r conn; do
        [ -z "\$conn" ] && continue
        ssid=\$(nmcli -t -f 802-11-wireless.ssid con show "\$conn" 2>/dev/null | cut -d: -f2)
        [ -z "\$ssid" ] && continue
        if echo "\$visible" | grep -qFx "\$ssid"; then
            logger -t pi-fallback-ap "Known network '\$ssid' in range — switching from AP"
            nmcli con down "\$AP_CONN" 2>/dev/null || true
            nmcli con up "\$conn" 2>/dev/null && return 0
        fi
    done <<< "\$known"
    return 1
}

if client_active; then
    if ap_active; then
        logger -t pi-fallback-ap "Client WiFi active — stopping fallback AP"
        nmcli con down "\$AP_CONN" 2>/dev/null || true
    fi
else
    if ap_active; then
        scan_for_known
    else
        logger -t pi-fallback-ap "No client WiFi — starting fallback AP"
        nmcli con up "\$AP_CONN" 2>/dev/null || true
    fi
fi
WDEOF
chmod +x "$WATCHDOG"

# ── 3. Systemd service + timer ────────────────────────────────
log "Installing systemd service and timer"
cat > "$SERVICE" <<UNITEOF
[Unit]
Description=Pi fallback AP watchdog
After=network.target NetworkManager.service
Requires=NetworkManager.service

[Service]
Type=oneshot
ExecStart=$WATCHDOG
UNITEOF

cat > "$TIMER" <<TIMEREOF
[Unit]
Description=Pi fallback AP watchdog timer

[Timer]
OnBootSec=45s
OnUnitActiveSec=30s
Unit=pi-fallback-ap.service

[Install]
WantedBy=timers.target
TIMEREOF

systemctl daemon-reload
systemctl enable --now pi-fallback-ap.timer

log "Done."
echo
echo "  SSID:     $AP_SSID"
echo "  Password: $AP_PASS"
echo "  Pi IP:    ${AP_IP%/*}"
echo
echo "  Watchdog runs every 30 s. When the Pi has no known WiFi, it"
echo "  brings up the AP. Join '$AP_SSID' from your laptop and SSH to"
echo "  ${AP_IP%/*} — no internet or Tailscale required."
echo
echo "  Check status:   systemctl status pi-fallback-ap.timer"
echo "  Watchdog log:   journalctl -t pi-fallback-ap -f"
