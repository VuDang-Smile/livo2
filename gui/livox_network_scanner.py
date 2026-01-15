#!/usr/bin/env python3
"""
Livox Network Scanner
Module to scan for Livox MID 360 devices in LAN network
Adapted from smile-lidar-recorder with connectivity scoring
"""

import socket
import threading
import time
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
import ipaddress


class LivoxNetworkScanner:
    """Class to scan for Livox devices in network"""
    
    def __init__(self, translator=None):
        self.translator = translator
        # Các port đặc trưng của Livox MID 360 (theo config file)
        self.livox_ports = {
            # Ports chính của Livox MID360
            56100: "Livox Command/Data Port (UDP)",
            56200: "Livox Push Message Port (UDP)", 
            56300: "Livox Point Data Port (UDP)",
            56400: "Livox IMU Data Port (UDP)",
            56500: "Livox Log Data Port (UDP)",
            # Ports phụ trợ
            56101: "Livox Command Port (TCP)", 
            80: "HTTP Web Interface",
            443: "HTTPS Web Interface",
            22: "SSH",
            23: "Telnet",
            8080: "Alternative HTTP",
            9090: "Livox Web Interface"
        }
        
        self.found_devices = []
        self.is_scanning = False
        self._scan_lock = threading.Lock()  # Lock để đảm bảo thread-safe
        
    def ping_host(self, ip, timeout=1):
        """Ping một host để kiểm tra xem có online không"""
        try:
            result = subprocess.run(['ping', '-c', '1', '-W', str(timeout), str(ip)], 
                                  capture_output=True, text=True, timeout=timeout+2)
            return result.returncode == 0
        except subprocess.TimeoutExpired:
            return False
        except Exception:
            return False
    
    def scan_tcp_port(self, ip, port, timeout=1):
        """Quét một TCP port cụ thể"""
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            result = sock.connect_ex((str(ip), port))
            return result == 0
        except Exception:
            return False
        finally:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
    
    def scan_udp_port(self, ip, port, timeout=1):
        """Quét UDP port (cho port 56100 của Livox)"""
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(timeout)
            sock.sendto(b"test", (str(ip), port))
            return True
        except Exception:
            return False
        finally:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
    
    def get_hostname(self, ip):
        """Lấy hostname của một IP"""
        try:
            hostname = socket.gethostbyaddr(str(ip))[0]
            return hostname
        except:
            return "Unknown"
    
    def get_mac_address(self, ip):
        """Lấy MAC address của một IP"""
        try:
            result = subprocess.run(['arp', '-n', str(ip)], 
                                  capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                lines = result.stdout.split('\n')
                for line in lines:
                    if str(ip) in line:
                        parts = line.split()
                        if len(parts) >= 3:
                            return parts[2]
            return "Unknown"
        except subprocess.TimeoutExpired:
            return "Unknown"
        except Exception:
            return "Unknown"
    
    def get_local_network_info(self):
        """Lấy thông tin mạng local của máy hiện tại"""
        try:
            # Lấy IP và interface của máy hiện tại
            result = subprocess.run(['ip', 'route', 'get', '8.8.8.8'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                lines = result.stdout.split('\n')
                for line in lines:
                    if 'src' in line and 'dev' in line:
                        parts = line.split()
                        local_ip = None
                        interface = None
                        
                        for i, part in enumerate(parts):
                            if part == 'src' and i + 1 < len(parts):
                                local_ip = parts[i + 1]
                            if part == 'dev' and i + 1 < len(parts):
                                interface = parts[i + 1]
                        
                        if local_ip and interface:
                            # Lấy subnet mask
                            result2 = subprocess.run(['ip', 'addr', 'show', interface], 
                                                    capture_output=True, text=True, timeout=5)
                            if result2.returncode == 0:
                                for line2 in result2.stdout.split('\n'):
                                    if 'inet ' in line2 and local_ip in line2:
                                        # Extract subnet
                                        parts2 = line2.split()
                                        for part2 in parts2:
                                            if '/' in part2 and 'inet' not in part2:
                                                subnet = part2.split('/')[1]
                                                network = ipaddress.ip_network(f"{local_ip}/{subnet}", strict=False)
                                                return {
                                                    'local_ip': local_ip,
                                                    'interface': interface,
                                                    'network': str(network),
                                                    'subnet_mask': subnet
                                                }
            return None
        except subprocess.TimeoutExpired:
            return None
        except Exception as e:
            return None
    
    def check_device_in_arp_table(self, ip):
        """Kiểm tra xem thiết bị có trong ARP table của máy mình không"""
        try:
            result = subprocess.run(['arp', '-n', str(ip)], 
                                  capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                # Nếu có kết quả và không phải "no entry", thiết bị đã từng giao tiếp
                if 'no entry' not in result.stdout.lower():
                    # Parse MAC address từ ARP table
                    lines = result.stdout.split('\n')
                    for line in lines:
                        if str(ip) in line:
                            parts = line.split()
                            if len(parts) >= 3:
                                mac = parts[2]
                                # Kiểm tra MAC có hợp lệ không
                                if mac and mac.lower() != 'incomplete':
                                    return {
                                        'in_arp': True,
                                        'mac': mac,
                                        'status': 'resolved'
                                    }
            return {'in_arp': False, 'mac': None, 'status': 'not_found'}
        except subprocess.TimeoutExpired:
            return {'in_arp': False, 'mac': None, 'status': 'error'}
        except Exception as e:
            return {'in_arp': False, 'mac': None, 'status': 'error'}
    
    def check_same_subnet(self, ip, local_network_info):
        """Kiểm tra xem thiết bị có cùng subnet với máy mình không"""
        if not local_network_info:
            return False
        
        try:
            device_ip = ipaddress.ip_address(str(ip))
            local_network = ipaddress.ip_network(local_network_info['network'], strict=False)
            return device_ip in local_network
        except:
            return False
    
    def measure_latency(self, ip, count=3):
        """Đo latency đến thiết bị (thiết bị trực tiếp thường có latency thấp hơn)"""
        try:
            latencies = []
            for _ in range(count):
                if not self.is_scanning:
                    break
                start_time = time.time()
                result = subprocess.run(['ping', '-c', '1', '-W', '1', str(ip)], 
                                      capture_output=True, text=True, timeout=2)
                if result.returncode == 0:
                    # Parse time từ ping output
                    for line in result.stdout.split('\n'):
                        if 'time=' in line:
                            try:
                                time_part = line.split('time=')[1].split()[0]
                                latency = float(time_part)
                                latencies.append(latency)
                            except:
                                pass
                time.sleep(0.1)
            
            if latencies:
                return {
                    'avg_latency': sum(latencies) / len(latencies),
                    'min_latency': min(latencies),
                    'max_latency': max(latencies),
                    'count': len(latencies)
                }
            return None
        except subprocess.TimeoutExpired:
            return None
        except Exception:
            return None
    
    def check_direct_route(self, ip, local_network_info):
        """Kiểm tra xem có route trực tiếp đến thiết bị không (không qua gateway)"""
        if not local_network_info:
            return False
        
        try:
            # Kiểm tra route đến IP
            result = subprocess.run(['ip', 'route', 'get', str(ip)], 
                                  capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                output = result.stdout.lower()
                # Route trực tiếp thường không có "via" (gateway)
                # và có "dev" với interface local
                if 'via' not in output and local_network_info['interface'] in output:
                    return True
            return False
        except subprocess.TimeoutExpired:
            return False
        except Exception:
            return False
    
    def check_device_connectivity_score(self, ip, device_info):
        """Tính điểm kết nối để xác định thiết bị nào được kết nối trực tiếp"""
        def t(key, default=None, **kwargs):
            """Helper to get translation"""
            if self.translator:
                msg = self.translator.get(key, default or key)
                for k, v in kwargs.items():
                    msg = msg.replace(f'{{{k}}}', str(v))
                return msg
            return default or key
        
        score = 0
        reasons = []
        
        # Lấy thông tin mạng local
        local_network_info = self.get_local_network_info()
        
        # 1. Kiểm tra ARP table (điểm cao nhất - 40 điểm)
        arp_info = self.check_device_in_arp_table(ip)
        if arp_info['in_arp'] and arp_info['status'] == 'resolved':
            score += 40
            reasons.append(t('log.arp_table_resolved', '✅ In ARP table (direct communication)'))
        elif arp_info['in_arp']:
            score += 20
            reasons.append(t('log.arp_table_unresolved', '⚠️ In ARP table but not resolved'))
        
        # 2. Kiểm tra cùng subnet (30 điểm)
        if local_network_info and self.check_same_subnet(ip, local_network_info):
            score += 30
            reasons.append(t('log.same_subnet', '✅ Same subnet ({network})', network=local_network_info['network']))
        else:
            reasons.append(t('log.different_subnet', '❌ Different subnet'))
        
        # 3. Kiểm tra route trực tiếp (20 điểm)
        if local_network_info and self.check_direct_route(ip, local_network_info):
            score += 20
            reasons.append(t('log.direct_route', '✅ Direct route (no gateway)'))
        else:
            reasons.append(t('log.route_via_gateway', '⚠️ Route via gateway or unknown'))
        
        # 4. Kiểm tra latency (10 điểm)
        latency_info = self.measure_latency(ip, count=3)
        if latency_info:
            avg_latency = latency_info['avg_latency']
            # Latency < 1ms thường là kết nối trực tiếp
            if avg_latency < 1.0:
                score += 10
                reasons.append(t('log.latency_very_low', '✅ Very low latency ({latency:.2f}ms) - direct connection', latency=avg_latency))
            elif avg_latency < 5.0:
                score += 5
                reasons.append(t('log.latency_low', '⚠️ Low latency ({latency:.2f}ms)', latency=avg_latency))
            else:
                reasons.append(t('log.latency_high', '❌ High latency ({latency:.2f}ms) - may be via switch/router', latency=avg_latency))
        
        return {
            'score': score,
            'reasons': reasons,
            'arp_info': arp_info,
            'latency_info': latency_info,
            'same_subnet': local_network_info and self.check_same_subnet(ip, local_network_info) if local_network_info else False,
            'direct_route': local_network_info and self.check_direct_route(ip, local_network_info) if local_network_info else False
        }
    
    def check_livox_http_response(self, ip):
        """Kiểm tra HTTP response để xác nhận thiết bị Livox"""
        try:
            import requests
            for port in [80, 443, 8080, 9090]:
                try:
                    protocol = 'https' if port in [443, 9090] else 'http'
                    url = f"{protocol}://{ip}:{port}"
                    response = requests.get(url, timeout=3)
                    if response.status_code == 200:
                        content = response.text.lower()
                        # Tìm từ khóa Livox trong nội dung web
                        livox_keywords = ['livox', 'mid360', 'lidar', 'livox mid360', 'livox mid 360']
                        keyword_count = sum(1 for keyword in livox_keywords if keyword in content)
                        
                        # Kiểm tra thêm các pattern đặc trưng của Livox
                        if 'livox' in content and ('mid360' in content or 'mid 360' in content):
                            return True
                        elif keyword_count >= 2:
                            return True
                except:
                    pass
            return False
        except ImportError:
            return False
    
    def check_livox_udp_signature(self, ip):
        """Kiểm tra UDP signature đặc trưng của Livox"""
        sock = None
        try:
            # Kiểm tra port 56100 với packet đặc trưng của Livox
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(2)
            
            # Gửi packet test đặc trưng của Livox MID360
            test_packet = b'\x00\x01\x02\x03'
            sock.sendto(test_packet, (str(ip), 56100))
            
            # Thử nhận response (một số thiết bị Livox sẽ phản hồi)
            try:
                sock.settimeout(1)
                response, addr = sock.recvfrom(1024)
                if response and len(response) > 0:
                    # Kiểm tra response pattern đặc trưng của Livox
                    if len(response) >= 4:
                        # Kiểm tra magic bytes hoặc pattern đặc trưng
                        if (response[0] == 0x00 and response[1] == 0x01) or \
                           (response[0] == 0x01 and response[1] == 0x00) or \
                           (response[0] == 0x02 and response[1] == 0x03):
                            return True
                    
                    # Kiểm tra response size đặc trưng của Livox
                    if len(response) in [4, 8, 16, 32, 64]:
                        return True
                        
            except Exception:
                pass
            
            return False
        except Exception:
            return False
        finally:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
    
    def check_livox_hostname_pattern(self, hostname):
        """Kiểm tra hostname pattern đặc trưng của Livox"""
        if not hostname or hostname.lower() == "unknown":
            # Hostname "Unknown" là đặc trưng của thiết bị Livox MID360 thật!
            return True
        
        hostname_lower = hostname.lower()
        
        # Thiết bị Livox thật thường có hostname đặc trưng
        # Loại bỏ các hostname phổ biến của máy tính thông thường
        common_pc_hostnames = [
            'system-product-name', 'desktop', 'laptop', 'pc', 'computer',
            'ubuntu', 'debian', 'fedora', 'centos', 'windows', 'mac',
            'raspberry', 'pi', 'nano', 'zero', 'router', 'gateway'
        ]
        
        # Nếu hostname chứa từ khóa máy tính thông thường, không phải Livox
        if any(keyword in hostname_lower for keyword in common_pc_hostnames):
            return False
        
        # Thiết bị Livox thật thường có hostname ngắn hoặc chứa số
        if len(hostname) <= 10 or any(char.isdigit() for char in hostname):
            return True
        
        # Kiểm tra từ khóa Livox trong hostname
        livox_keywords = ['livox', 'mid', 'lidar', '360', 'sensor']
        if any(keyword in hostname_lower for keyword in livox_keywords):
            return True
        
        return False
    
    def check_livox_device(self, ip):
        """Kiểm tra xem một IP có phải là thiết bị Livox không"""
        device_info = {
            'ip': str(ip),
            'hostname': self.get_hostname(ip),
            'mac': self.get_mac_address(ip),
            'open_ports': [],
            'is_livox': False,
            'confidence': 0,
            'livox_signature': False,
            'http_response': None
        }
        
        # Kiểm tra ping trước - chỉ tiếp tục nếu ping được
        if not self.ping_host(ip, timeout=2):
            device_info['confidence'] = 0
            return device_info
        
        # Kiểm tra các port đặc trưng của Livox MID360
        livox_udp_ports = [56100, 56200, 56300, 56400, 56500]
        
        for port in self.livox_ports.keys():
            if port in livox_udp_ports:
                if self.scan_udp_port(ip, port):
                    device_info['open_ports'].append(port)
            else:
                if self.scan_tcp_port(ip, port):
                    device_info['open_ports'].append(port)
        
        # Kiểm tra HTTP response để xác nhận thiết bị Livox
        device_info['http_response'] = self.check_livox_http_response(ip)
        
        # Kiểm tra UDP signature đặc trưng của Livox
        device_info['livox_signature'] = self.check_livox_udp_signature(ip)
        
        # Kiểm tra hostname pattern đặc trưng của Livox
        device_info['hostname_pattern'] = self.check_livox_hostname_pattern(device_info['hostname'])
        
        # Tính điểm confidence
        confidence = 0
        
        livox_udp_signature_ports = [56100, 56200, 56300, 56400, 56500]
        livox_tcp_signature_ports = [56101]
        web_ports = [80, 443, 8080, 9090]
        
        # Điểm cho port UDP đặc trưng Livox
        livox_udp_ports_found = 0
        for port in livox_udp_signature_ports:
            if port in device_info['open_ports']:
                livox_udp_ports_found += 1
                if port == 56100:
                    confidence += 20
                elif port == 56200:
                    confidence += 15
                elif port == 56300:
                    confidence += 15
                elif port == 56400:
                    confidence += 10
                elif port == 56500:
                    confidence += 10
        
        # Chỉ coi là Livox nếu có ít nhất 2 port UDP đặc trưng
        if livox_udp_ports_found < 2:
            device_info['confidence'] = 0
            return device_info
        
        # Điểm cho port TCP đặc trưng Livox
        for port in livox_tcp_signature_ports:
            if port in device_info['open_ports']:
                confidence += 15
        
        # Điểm cho web interface
        for port in web_ports:
            if port in device_info['open_ports']:
                confidence += 5
        
        # Điểm cho SSH/Telnet
        if 22 in device_info['open_ports'] or 23 in device_info['open_ports']:
            confidence += 3
        
        # Kiểm tra hostname
        hostname_lower = device_info['hostname'].lower()
        hostname_keywords = ['livox', 'mid', 'lidar', '360']
        hostname_matches = sum(1 for keyword in hostname_keywords if keyword in hostname_lower)
        
        if hostname_matches > 0:
            confidence += min(hostname_matches * 10, 20)
        
        # Bonus điểm cho HTTP response xác nhận Livox
        if device_info['http_response']:
            confidence += 25
        
        # Bonus điểm cho UDP signature xác nhận Livox
        if device_info['livox_signature']:
            confidence += 30
        
        # Bonus điểm cho hostname pattern
        if device_info['hostname_pattern']:
            if device_info['hostname'].lower() == "unknown":
                confidence += 50
            else:
                confidence += 30
        else:
            confidence -= 20
        
        device_info['confidence'] = confidence
        
        # Logic xác nhận Livox
        is_livox = False
        
        if livox_udp_ports_found >= 2:
            if confidence >= 50:
                is_livox = True
            elif device_info['http_response'] or device_info['livox_signature']:
                if confidence >= 40:
                    is_livox = True
            elif device_info['hostname'].lower() == "unknown" and confidence >= 30:
                is_livox = True
        
        device_info['is_livox'] = is_livox
        
        # Sau khi xác định là Livox, kiểm tra kết nối trực tiếp
        if device_info['is_livox']:
            connectivity = self.check_device_connectivity_score(ip, device_info)
            device_info['connectivity_score'] = connectivity['score']
            device_info['connectivity_reasons'] = connectivity['reasons']
            device_info['is_directly_connected'] = connectivity['score'] >= 50
            device_info['arp_info'] = connectivity['arp_info']
            device_info['latency_info'] = connectivity['latency_info']
            device_info['same_subnet'] = connectivity['same_subnet']
            device_info['direct_route'] = connectivity['direct_route']
        
        return device_info
    
    def get_current_network_range(self):
        """Tự động phát hiện dải mạng hiện tại"""
        try:
            # Lấy thông tin mạng hiện tại
            result = subprocess.run(['ip', 'route', 'get', '8.8.8.8'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                lines = result.stdout.split('\n')
                for line in lines:
                    if 'src' in line:
                        parts = line.split()
                        for i, part in enumerate(parts):
                            if part == 'src':
                                if i + 1 < len(parts):
                                    current_ip = parts[i + 1]
                                    # Tạo dải mạng từ IP hiện tại
                                    ip_obj = ipaddress.ip_address(current_ip)
                                    if ip_obj.is_private:
                                        # Tạo /24 network từ IP hiện tại
                                        network = ipaddress.ip_network(f"{current_ip}/24", strict=False)
                                        return str(network)
            return "192.168.1.0/24"  # Fallback
        except:
            return "192.168.1.0/24"  # Fallback
    
    def scan_current_network(self, callback=None):
        """Quét mạng Livox (192.168.1.0/24) - mạng riêng của Livox MID360"""
        # Livox MID360 thường ở mạng riêng 192.168.1.0/24
        # Quét mạng này bất kể mạng hiện tại của máy là gì
        livox_network = "192.168.1.0/24"
        current_range = self.get_current_network_range()
        
        if callback:
            callback(f"Current machine network: {current_range}")
            callback(f"Scanning Livox network: {livox_network} (Livox MID360 network)")
        
        self.scan_network_range(livox_network, callback)
    
    def scan_network_range(self, network_range="192.168.1.0/24", callback=None, stop_on_found=True):
        """Quét toàn bộ dải mạng
        
        Args:
            network_range: Dải mạng cần quét
            callback: Callback function để log
            stop_on_found: Nếu True, dừng khi tìm thấy thiết bị Livox (cho scan đơn lẻ)
        """
        # Chỉ set is_scanning = True nếu chưa được set (cho phép scan nhiều mạng)
        with self._scan_lock:
            if not self.is_scanning:
                self.is_scanning = True
        # Không reset found_devices nếu đang quét nhiều mạng
        if stop_on_found:
            with self._scan_lock:
                self.found_devices = []
        
        def scan_thread():
            try:
                if callback:
                    callback(f"Starting to scan network {network_range}...")
                
                # Lấy danh sách IP trong dải mạng
                network = ipaddress.ip_network(network_range)
                ip_list = [ip for ip in network.hosts()]

                # Chỉ giữ các IP dạng 192.168.1.1XX (100-199) theo yêu cầu (giống recorder)
                def _is_livox_candidate(ip_addr):
                    try:
                        parts = str(ip_addr).split('.')
                        if len(parts) != 4:
                            return False
                        # Chỉ chấp nhận subnet 192.168.1.x
                        if parts[0] != '192' or parts[1] != '168' or parts[2] != '1':
                            return False
                        last_octet = int(parts[3])
                        return 100 <= last_octet <= 199
                    except Exception:
                        return False

                filtered_ip_list = [ip for ip in ip_list if _is_livox_candidate(ip)]
                if callback:
                    callback(f"Filtered to {len(filtered_ip_list)} candidates in 192.168.1.100-199 range")
                
                if callback:
                    callback(f"Pinging {len(filtered_ip_list)} IPs...")
                
                # Ping tất cả IP để tìm host online
                online_hosts = []
                executor_ping = None
                try:
                    executor_ping = ThreadPoolExecutor(max_workers=50)
                    future_to_ip = {executor_ping.submit(self.ping_host, ip): ip for ip in filtered_ip_list}
                    
                    for future in as_completed(future_to_ip):
                        if not self.is_scanning:
                            # Cancel remaining futures
                            for f in future_to_ip:
                                if not f.done():
                                    f.cancel()
                            break
                            
                        ip = future_to_ip[future]
                        try:
                            if future.result():
                                online_hosts.append(ip)
                                if callback:
                                    callback(f"✅ Host online: {ip}")
                        except Exception as e:
                            pass
                finally:
                    if executor_ping is not None:
                        executor_ping.shutdown(wait=True, cancel_futures=True)
                
                if callback:
                    callback(f"Found {len(online_hosts)} online hosts. Scanning in detail...")
                
                # Quét chi tiết từng host online
                executor_scan = None
                try:
                    executor_scan = ThreadPoolExecutor(max_workers=20)
                    future_to_ip = {executor_scan.submit(self.check_livox_device, ip): ip for ip in online_hosts}
                    
                    for future in as_completed(future_to_ip):
                        if not self.is_scanning:
                            # Cancel remaining futures
                            for f in future_to_ip:
                                if not f.done():
                                    f.cancel()
                            break
                            
                        ip = future_to_ip[future]
                        try:
                            device_info = future.result()
                            with self._scan_lock:
                                self.found_devices.append(device_info)
                            
                            if callback:
                                if device_info['is_livox']:
                                    def t(key, default=None, **kwargs):
                                        if self.translator:
                                            msg = self.translator.get(key, default or key)
                                            for k, v in kwargs.items():
                                                msg = msg.replace(f'{{{k}}}', str(v))
                                            return msg
                                        return default or key
                                    connection_status = t('label.direct_connection', '🔌 Direct') if device_info.get('is_directly_connected', False) else t('label.network_connection', '🌐 Network')
                                    callback(f"🎯 LIVOX DEVICE: {device_info['ip']} {connection_status} (confidence: {device_info['confidence']}%, connectivity: {device_info.get('connectivity_score', 0)})")
                                else:
                                    callback(f"📱 Other device: {device_info['ip']} ({device_info['hostname']})")
                                    
                        except Exception as e:
                            if callback:
                                callback(f"❌ Error scanning {ip}: {e}")
                finally:
                    if executor_scan is not None:
                        executor_scan.shutdown(wait=True, cancel_futures=True)
                
                if callback:
                    with self._scan_lock:
                        livox_count = len([d for d in self.found_devices if d['is_livox']])
                        total_count = len(self.found_devices)
                    callback(f"✅ Completed! Found {livox_count} Livox devices out of {total_count} devices")
                    
            except Exception as e:
                if callback:
                    callback(f"❌ Scan error: {e}")
            finally:
                with self._scan_lock:
                    self.is_scanning = False
        
        threading.Thread(target=scan_thread, daemon=True).start()
    
    def stop_scanning(self):
        """Dừng quá trình quét an toàn"""
        with self._scan_lock:
            self.is_scanning = False
        # Đợi một chút để các threads có thể cleanup
        time.sleep(0.1)
    
    def get_livox_devices(self):
        """Lấy danh sách các thiết bị Livox đã tìm thấy"""
        with self._scan_lock:
            return [d for d in self.found_devices if d['is_livox']]
    
    def get_all_devices(self):
        """Lấy danh sách tất cả thiết bị đã tìm thấy"""
        with self._scan_lock:
            return self.found_devices.copy()  # Trả về copy để tránh race condition
