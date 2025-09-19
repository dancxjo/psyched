#!/usr/bin/env bash
# Script to manage systemd services for psyched modules

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMPLATE_FILE="${SCRIPT_DIR}/systemd/psyched-module.service.template"
SERVICES_DIR="/etc/systemd/system"
SERVICE_PREFIX="psyched-"

# Default values
USER="${SUDO_USER:-$(whoami)}"
GROUP="$(id -gn "${USER}")"
HOME_DIR="$(eval echo "~${USER}")"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "This script must be run with sudo"
        exit 1
    fi
}

get_enabled_modules() {
    local host_dir="${REPO_DIR}/hosts"
    local hostname="${1:-$(hostname)}"
    local enabled_dir="${host_dir}/${hostname}/enabled"
    
    if [[ ! -d "${enabled_dir}" ]]; then
        log_warning "No enabled modules found for host '${hostname}'"
        return 1
    fi
    
    # List all files/links in enabled directory, extract module names
    find "${enabled_dir}" -maxdepth 1 -type f -o -type l | while read -r file; do
        basename "$file" | sed 's/^[0-9]*-//'
    done
}

get_all_modules() {
    local modules_dir="${REPO_DIR}/modules"
    if [[ ! -d "${modules_dir}" ]]; then
        log_error "Modules directory not found: ${modules_dir}"
        return 1
    fi
    
    find "${modules_dir}" -maxdepth 1 -mindepth 1 -type d -exec basename {} \;
}

generate_service_file() {
    local module_name="$1"
    local output_file="$2"
    
    if [[ ! -f "${TEMPLATE_FILE}" ]]; then
        log_error "Template file not found: ${TEMPLATE_FILE}"
        exit 1
    fi
    
    # Replace template variables
    sed -e "s|{{MODULE_NAME}}|${module_name}|g" \
        -e "s|{{USER}}|${USER}|g" \
        -e "s|{{GROUP}}|${GROUP}|g" \
        -e "s|{{REPO_DIR}}|${REPO_DIR}|g" \
        -e "s|{{HOME_DIR}}|${HOME_DIR}|g" \
        -e "s|{{ROS_DOMAIN_ID}}|${ROS_DOMAIN_ID}|g" \
        "${TEMPLATE_FILE}" > "${output_file}"
}

install_service() {
    local module_name="$1"
    local service_name="${SERVICE_PREFIX}${module_name}.service"
    local service_file="${SERVICES_DIR}/${service_name}"
    local module_launch="${REPO_DIR}/modules/${module_name}/launch.sh"
    
    # Check if module exists
    if [[ ! -f "${module_launch}" ]]; then
        log_error "Module '${module_name}' not found or missing launch.sh"
        return 1
    fi
    
    # Make launch script executable
    chmod +x "${module_launch}"
    
    # Generate service file
    log_info "Installing service for module '${module_name}'"
    generate_service_file "${module_name}" "${service_file}"
    
    # Reload systemd and enable service
    systemctl daemon-reload
    systemctl enable "${service_name}"
    
    log_success "Service '${service_name}' installed and enabled"
}

uninstall_service() {
    local module_name="$1"
    local service_name="${SERVICE_PREFIX}${module_name}.service"
    local service_file="${SERVICES_DIR}/${service_name}"
    
    if [[ -f "${service_file}" ]]; then
        log_info "Uninstalling service for module '${module_name}'"
        
        # Stop and disable service
        systemctl stop "${service_name}" 2>/dev/null || true
        systemctl disable "${service_name}" 2>/dev/null || true
        
        # Remove service file
        rm -f "${service_file}"
        
        # Reload systemd
        systemctl daemon-reload
        
        log_success "Service '${service_name}' uninstalled"
    else
        log_warning "Service '${service_name}' not found"
    fi
}

start_service() {
    local module_name="$1"
    local service_name="${SERVICE_PREFIX}${module_name}.service"
    
    log_info "Starting service '${service_name}'"
    systemctl start "${service_name}"
    log_success "Service '${service_name}' started"
}

stop_service() {
    local module_name="$1"
    local service_name="${SERVICE_PREFIX}${module_name}.service"
    
    log_info "Stopping service '${service_name}'"
    systemctl stop "${service_name}"
    log_success "Service '${service_name}' stopped"
}

status_service() {
    local module_name="$1"
    local service_name="${SERVICE_PREFIX}${module_name}.service"
    
    systemctl status "${service_name}" --no-pager
}

list_services() {
    echo "Psyched systemd services:"
    systemctl list-units "${SERVICE_PREFIX}*.service" --no-pager --all
}

install_enabled_services() {
    local hostname="${1:-$(hostname)}"
    
    log_info "Installing services for enabled modules on host '${hostname}'"
    
    if ! get_enabled_modules "${hostname}" >/dev/null 2>&1; then
        log_error "No enabled modules found for host '${hostname}'"
        exit 1
    fi
    
    get_enabled_modules "${hostname}" | while read -r module; do
        install_service "${module}"
    done
    
    log_success "All enabled services installed"
}

uninstall_all_services() {
    log_info "Uninstalling all psyched services"
    
    # Find all psyched services
    systemctl list-units "${SERVICE_PREFIX}*.service" --no-pager --plain --no-legend | \
    awk '{print $1}' | \
    sed "s/^${SERVICE_PREFIX}//" | sed 's/\.service$//' | \
    while read -r module; do
        if [[ -n "${module}" ]]; then
            uninstall_service "${module}"
        fi
    done
    
    log_success "All psyched services uninstalled"
}

start_enabled_services() {
    local hostname="${1:-$(hostname)}"
    
    log_info "Starting services for enabled modules on host '${hostname}'"
    
    if ! get_enabled_modules "${hostname}" >/dev/null 2>&1; then
        log_error "No enabled modules found for host '${hostname}'"
        exit 1
    fi
    
    get_enabled_modules "${hostname}" | while read -r module; do
        start_service "${module}"
    done
    
    log_success "All enabled services started"
}

stop_all_services() {
    log_info "Stopping all psyched services"
    
    systemctl list-units "${SERVICE_PREFIX}*.service" --no-pager --plain --no-legend | \
    awk '{print $1}' | \
    sed "s/^${SERVICE_PREFIX}//" | sed 's/\.service$//' | \
    while read -r module; do
        if [[ -n "${module}" ]]; then
            stop_service "${module}"
        fi
    done
    
    log_success "All psyched services stopped"
}

show_help() {
    cat << EOF
Usage: $0 <command> [options]

Commands:
    install <module>        Install systemd service for a specific module
    uninstall <module>      Uninstall systemd service for a specific module
    start <module>          Start a specific module service
    stop <module>           Stop a specific module service
    status <module>         Show status of a specific module service
    
    install-enabled [host]  Install services for all enabled modules (default: current hostname)
    uninstall-all          Uninstall all psyched services
    start-enabled [host]   Start all enabled module services (default: current hostname)
    stop-all              Stop all psyched services
    
    list                   List all psyched services
    modules                List all available modules
    enabled [host]         List enabled modules for host (default: current hostname)
    
    help                   Show this help message

Examples:
    sudo $0 install-enabled              # Install services for enabled modules
    sudo $0 install voice               # Install service for voice module
    sudo $0 start-enabled               # Start all enabled services
    sudo $0 stop voice                  # Stop voice service
    sudo $0 status foot                 # Show status of foot service
    sudo $0 uninstall-all               # Remove all services
    $0 list                            # List services (no sudo needed)
    $0 modules                         # List all modules (no sudo needed)

Environment Variables:
    ROS_DOMAIN_ID          ROS domain ID (default: 0)
    USER                   User to run services as (auto-detected)

EOF
}

# Main script logic
case "${1:-help}" in
    install)
        check_root
        if [[ -z "${2:-}" ]]; then
            log_error "Module name required"
            show_help
            exit 1
        fi
        install_service "$2"
        ;;
    
    uninstall)
        check_root
        if [[ -z "${2:-}" ]]; then
            log_error "Module name required"
            show_help
            exit 1
        fi
        uninstall_service "$2"
        ;;
    
    start)
        check_root
        if [[ -z "${2:-}" ]]; then
            log_error "Module name required"
            show_help
            exit 1
        fi
        start_service "$2"
        ;;
    
    stop)
        check_root
        if [[ -z "${2:-}" ]]; then
            log_error "Module name required"
            show_help
            exit 1
        fi
        stop_service "$2"
        ;;
    
    status)
        if [[ -z "${2:-}" ]]; then
            log_error "Module name required"
            show_help
            exit 1
        fi
        status_service "$2"
        ;;
    
    install-enabled)
        check_root
        install_enabled_services "${2:-}"
        ;;
    
    uninstall-all)
        check_root
        uninstall_all_services
        ;;
    
    start-enabled)
        check_root
        start_enabled_services "${2:-}"
        ;;
    
    stop-all)
        check_root
        stop_all_services
        ;;
    
    list)
        list_services
        ;;
    
    modules)
        echo "Available modules:"
        if get_all_modules >/dev/null 2>&1; then
            get_all_modules | sort
        else
            echo "  (none found or modules directory missing)"
        fi
        ;;
    
    enabled)
        hostname="${2:-$(hostname)}"
        echo "Enabled modules for host '${hostname}':"
        if get_enabled_modules "${hostname}" >/dev/null 2>&1; then
            get_enabled_modules "${hostname}" | sort
        else
            echo "  (none found)"
        fi
        ;;
    
    help|--help|-h)
        show_help
        ;;
    
    *)
        log_error "Unknown command: $1"
        show_help
        exit 1
        ;;
esac