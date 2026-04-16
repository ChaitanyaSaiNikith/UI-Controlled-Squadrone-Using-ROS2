#!/usr/bin/env python3
"""Serve web_ui.html on port 8080.

Usage:
    cd ~/squadron_ros2_ws/web && python3 web_server.py
"""
import http.server
import socketserver
import os

PORT = 8080
DIR  = os.path.dirname(os.path.abspath(__file__))


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIR, **kwargs)

    def log_message(self, fmt, *args):  # quieter logging
        print(f"[web] {self.address_string()} {fmt % args}")


with socketserver.TCPServer(('', PORT), Handler) as httpd:
    print(f"Serving Squadron UI at http://localhost:{PORT}/web_ui.html")
    httpd.serve_forever()
