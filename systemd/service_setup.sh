#!/usr/bin/env bash

SERVICE_NAME="twist_to_ackermann.service"
SERVICE_FILE="systemd/${SERVICE_NAME}"
SERVICE_PATH="$(realpath "systemd/twist_to_ackermann.service")"

case "${1:-}" in
  install)
    systemctl --user link "$SERVICE_PATH"
    systemctl --user daemon-reload
    systemctl --user enable --now "$SERVICE_NAME"
    echo "Installed and started $SERVICE_NAME"
    sleep 1
    systemctl --user status "$SERVICE_NAME"
    ;;

  uninstall)
    systemctl --user disable --now "$SERVICE_NAME" || true
    systemctl --user daemon-reload
    systemctl --user reset-failed "$SERVICE_NAME" || true
    echo "Uninstalled $SERVICE_NAME"
    ;;

  *)
    echo "Usage: $0 {install|uninstall}"
    ;;
esac