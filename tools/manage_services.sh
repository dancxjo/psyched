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
    local debug_mode="${2:-false}"
    
    log_info "Starting service '${service_name}'"
    
    if [[ "${debug_mode}" == "true" ]]; then
        log_info "Debug mode: Starting service with detailed logging..."
        # Clear any previous journal entries for this service
        journalctl --vacuum-time=1s --quiet || true
        
        # Attempt to start the service
        if ! systemctl start "${service_name}"; then
            log_error "Service '${service_name}' failed to start"
            log_info "=== SYSTEMD STATUS ==="
            systemctl status "${service_name}" --no-pager --lines=20 || true
            log_info "=== RECENT JOURNAL ENTRIES ==="
            journalctl -u "${service_name}" --no-pager --lines=50 --since="1 minute ago" || true
            log_info "=== ENVIRONMENT CHECK ==="
            log_info "Checking if environment setup works manually..."
            sudo -u "${USER}" bash -c "cd '${REPO_DIR}' && eval \"\$(SETUP_ENV_MODE=print ./tools/setup_env.sh)\" && echo 'Environment setup successful'" || log_error "Environment setup failed"
            log_info "=== MODULE LAUNCH SCRIPT CHECK ==="
            local launch_script="${REPO_DIR}/modules/${module_name}/launch.sh"
            if [[ -f "${launch_script}" ]]; then
                log_info "Launch script exists: ${launch_script}"
                if [[ -x "${launch_script}" ]]; then
                    log_info "Launch script is executable"
                else
                    log_error "Launch script is NOT executable"
                fi
            else
                log_error "Launch script not found: ${launch_script}"
            fi
            return 1
        fi
    else
        systemctl start "${service_name}"
    fi
    
    # Wait a moment and check if the service is actually running
    sleep 2
    if systemctl is-active "${service_name}" >/dev/null 2>&1; then
        log_success "Service '${service_name}' started successfully"
        if [[ "${debug_mode}" == "true" ]]; then
            log_info "Service is active and running"
            log_info "Recent logs:"
            journalctl -u "${service_name}" --no-pager --lines=10 --since="30 seconds ago" || true
        fi
    else
        log_error "Service '${service_name}' failed to stay running"
        if [[ "${debug_mode}" == "true" ]]; then
            log_info "=== FAILURE ANALYSIS ==="
            systemctl status "${service_name}" --no-pager --lines=10 || true
            log_info "=== RECENT LOGS ==="
            journalctl -u "${service_name}" --no-pager --lines=30 --since="1 minute ago" || true
        fi
        return 1
    fi
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

show_service_logs() {
    local module_name="$1"
    local lines="${2:-20}"
    local service_name="${SERVICE_PREFIX}${module_name}.service"
    
    log_info "Showing recent logs for service '${service_name}' (${lines} lines):"
    journalctl -u "${service_name}" --no-pager --lines="${lines}" --since="1 hour ago"
}

diagnose_service() {
    local module_name="$1"
    local service_name="${SERVICE_PREFIX}${module_name}.service"
    local launch_script="${REPO_DIR}/modules/${module_name}/launch.sh"
    
    echo "=== PSYCHED SERVICE DIAGNOSTIC: ${module_name} ==="
    echo
    
    # Check if service exists
    if systemctl list-unit-files "${service_name}" --no-pager --no-legend | grep -q "${service_name}"; then
        echo "✓ Service file exists"
    else
        echo "✗ Service file does not exist"
        echo "  Run: sudo make install-services"
        return 1
    fi
    
    # Check service status
    echo
    echo "=== SERVICE STATUS ==="
    systemctl status "${service_name}" --no-pager --lines=10 || true
    
    # Check launch script
    echo
    echo "=== LAUNCH SCRIPT CHECK ==="
    if [[ -f "${launch_script}" ]]; then
        echo "✓ Launch script exists: ${launch_script}"
        if [[ -x "${launch_script}" ]]; then
            echo "✓ Launch script is executable"
        else
            echo "✗ Launch script is not executable"
            echo "  Fix with: chmod +x ${launch_script}"
        fi
    else
        echo "✗ Launch script not found: ${launch_script}"
    fi
    
    # Test environment setup
    echo
    echo "=== ENVIRONMENT SETUP TEST ==="
    if sudo -u "${USER}" bash -c "cd '${REPO_DIR}' && eval \"\$(SETUP_ENV_MODE=print ./tools/setup_env.sh)\" >/dev/null 2>&1"; then
        echo "✓ Environment setup works"
        sudo -u "${USER}" bash -c "cd '${REPO_DIR}' && eval \"\$(SETUP_ENV_MODE=print ./tools/setup_env.sh)\" && echo \"  ROS_DISTRO: \${ROS_DISTRO:-unset}\" && echo \"  ROS installation: \$(ros2 --version 2>/dev/null || echo 'NOT FOUND')\""
    else
        echo "✗ Environment setup failed"
        echo "  Trying to identify the issue..."
        sudo -u "${USER}" bash -c "cd '${REPO_DIR}' && eval \"\$(SETUP_ENV_MODE=print ./tools/setup_env.sh)\"" || true
    fi
    
    # Check module dependencies
    echo
    echo "=== MODULE DEPENDENCIES ==="
    local module_dir="${REPO_DIR}/modules/${module_name}"
    if [[ -f "${module_dir}/setup.sh" ]]; then
        echo "✓ Module has setup.sh script"
        echo "  Make sure to run: ./modules/${module_name}/setup.sh"
    else
        echo "? No setup.sh script found for module"
    fi
    
    # Check recent logs
    echo
    echo "=== RECENT LOGS (last 30 lines) ==="
    if journalctl -u "${service_name}" --no-pager --lines=30 --since="1 hour ago" >/dev/null 2>&1; then
        journalctl -u "${service_name}" --no-pager --lines=30 --since="1 hour ago"
    else
        echo "No recent logs found for this service"
    fi
    
    echo
    echo "=== DIAGNOSIS COMPLETE ==="
    echo "If issues persist, try:"
    echo "  sudo make start-services-debug"
    echo "  sudo ./tools/manage_services.sh logs ${module_name} 50"
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
    local debug_mode="${2:-false}"
    
    log_info "Starting services for enabled modules on host '${hostname}'"
    
    if ! get_enabled_modules "${hostname}" >/dev/null 2>&1; then
        log_error "No enabled modules found for host '${hostname}'"
        exit 1
    fi
    
    local failed_services=()
    
    get_enabled_modules "${hostname}" | while read -r module; do
        if ! start_service "${module}" "${debug_mode}"; then
            failed_services+=("${module}")
            if [[ "${debug_mode}" == "false" ]]; then
                log_error "Service '${SERVICE_PREFIX}${module}.service' failed to start"
                log_info "Run with debug mode for detailed diagnostics: sudo make start-services-debug"
            fi
        fi
    done
    
    if [[ ${#failed_services[@]} -gt 0 ]]; then
        log_error "Failed to start ${#failed_services[@]} service(s): ${failed_services[*]}"
        exit 1
    fi
    
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
    start-debug <module>    Start a specific module service with detailed debugging
    stop <module>           Stop a specific module service
    status <module>         Show status of a specific module service
    
    install-enabled [host]  Install services for all enabled modules (default: current hostname)
    uninstall-all          Uninstall all psyched services
    start-enabled [host]   Start all enabled module services (default: current hostname)
    start-enabled-debug [host] Start all enabled services with detailed debugging
    stop-all              Stop all psyched services
    
    diagnose <module>      Run comprehensive diagnostics on a specific module service
    logs <module> [lines]  Show recent logs for a module service (default: 20 lines)
    list                   List all psyched services
    modules                List all available modules
    enabled [host]         List enabled modules for host (default: current hostname)
    
    help                   Show this help message

Examples:
    sudo $0 install-enabled              # Install services for enabled modules
    sudo $0 install voice               # Install service for voice module
    sudo $0 start-enabled               # Start all enabled services
    sudo $0 start-enabled-debug         # Start services with detailed debugging
    sudo $0 start-debug voice           # Start voice service with debug output
    sudo $0 stop voice                  # Stop voice service
    sudo $0 status foot                 # Show status of foot service
    $0 diagnose voice                  # Run diagnostics on voice service (no sudo)
    $0 logs voice 50                   # Show last 50 log lines for voice service
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
        start_enabled_services "${2:-}" "false"
        ;;
    
    start-enabled-debug)
        check_root
        start_enabled_services "${2:-}" "true"
        ;;
    
    start-debug)
        check_root
        if [[ -z "${2:-}" ]]; then
            log_error "Module name required"
            show_help
            exit 1
        fi
        start_service "$2" "true"
        ;;
    
    diagnose)
        if [[ -z "${2:-}" ]]; then
            log_error "Module name required for diagnosis"
            show_help
            exit 1
        fi
        diagnose_service "$2"
        ;;
    
    logs)
        if [[ -z "${2:-}" ]]; then
            log_error "Module name required"
            show_help
            exit 1
        fi
        show_service_logs "$2" "${3:-20}"
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