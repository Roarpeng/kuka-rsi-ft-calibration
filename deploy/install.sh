#!/usr/bin/env bash
# KUKA RSI 上位机 Ubuntu 部署脚本（需要 sudo）
# 用法：在项目根目录执行  sudo bash deploy/install.sh
set -euo pipefail

INSTALL_DIR=/opt/kuka-rsi
SERVICE_NAME=kuka-rsi
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ $EUID -ne 0 ]]; then
    echo "请用 sudo 运行：sudo bash deploy/install.sh" >&2
    exit 1
fi

echo "==> 安装到 ${INSTALL_DIR}"
mkdir -p "${INSTALL_DIR}/data"
cp "${SRC_DIR}"/*.py "${INSTALL_DIR}/"
cp -r "${SRC_DIR}/web_static" "${INSTALL_DIR}/"
# 配置与标定结果：仅当目标不存在时拷贝，避免覆盖现场已标定的数据
for f in ft_calibration_config.json ft_calibration.json ft_calibration_samples.json; do
    if [[ -f "${SRC_DIR}/${f}" && ! -f "${INSTALL_DIR}/${f}" ]]; then
        cp "${SRC_DIR}/${f}" "${INSTALL_DIR}/"
    fi
done

echo "==> 安装 systemd 服务 ${SERVICE_NAME}"
cp "${SRC_DIR}/deploy/kuka-rsi.service" "/etc/systemd/system/${SERVICE_NAME}.service"
systemctl daemon-reload
systemctl enable --now "${SERVICE_NAME}"

echo
echo "==> 完成。常用命令："
echo "  查看状态：  systemctl status ${SERVICE_NAME}"
echo "  查看日志：  journalctl -u ${SERVICE_NAME} -f"
echo "  重启：      systemctl restart ${SERVICE_NAME}"
echo "  切换模式：  编辑 /etc/systemd/system/${SERVICE_NAME}.service 的 RSI_MODE 后"
echo "              systemctl daemon-reload && systemctl restart ${SERVICE_NAME}"
echo
echo "Web 监控：http://<本机IP>:8080 （局域网开放，无登录）"
echo
echo "注意：本机网口需配置静态 IP 192.168.2.250（机器人 RSI XML 的 IP_NUMBER），"
echo "      例如：sudo nmcli con mod <连接名> ipv4.addresses 192.168.2.250/24 ipv4.method manual"
