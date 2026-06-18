#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import json
import math
import threading

try:
    from http.server import BaseHTTPRequestHandler, HTTPServer
    from socketserver import ThreadingMixIn
except ImportError:
    from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
    from SocketServer import ThreadingMixIn

import rospy
import tf2_ros
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Bool, String

from cluster_msgs.msg import FollowerStatus, LeaderCmd
from cluster_msgs.srv import SetFormation, SetMode

try:
    unicode
except NameError:
    unicode = str


INDEX_HTML = r"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>LIMO Cluster Visualizer</title>
  <style>
    html, body { height: 100%; margin: 0; font-family: Arial, sans-serif; background:#101418; color:#e7edf3; }
    .app { display:flex; height:100vh; overflow:hidden; }
    .main { flex:1; display:flex; flex-direction:column; min-width:0; }
    .bar { display:flex; gap:8px; flex-wrap:wrap; padding:10px 12px; border-bottom:1px solid #26313b; background:#121920; align-items:center; }
    button { background:#1c2630; color:#e7edf3; border:1px solid #31404d; padding:8px 10px; cursor:pointer; border-radius:4px; }
    button:hover { background:#243140; }
    button:active { background:#314153; }
    .content { display:flex; flex:1; min-height:0; }
    .canvas-wrap { flex:1; min-width:0; position:relative; background:#0f1419; }
    canvas { width:100%; height:100%; display:block; }
    .panel { width:320px; max-width:42vw; border-left:1px solid #26313b; background:#0f151b; padding:12px; box-sizing:border-box; overflow:auto; }
    .section { margin-bottom:14px; }
    .title { font-size:12px; letter-spacing:0.08em; color:#93a4b4; margin-bottom:8px; text-transform:uppercase; }
    .row { display:flex; justify-content:space-between; gap:10px; font-size:13px; padding:4px 0; border-bottom:1px solid rgba(255,255,255,0.05); }
    .muted { color:#8a98a8; }
    .robot { margin-bottom:10px; padding-bottom:10px; border-bottom:1px solid rgba(255,255,255,0.06); }
    .robot:last-child { border-bottom:none; }
    .status-line { display:flex; justify-content:space-between; gap:8px; }
    .badge { display:inline-block; padding:2px 6px; border-radius:4px; background:#1f2b36; color:#c8d3de; font-size:12px; }
    .badge.ok { background:#184b34; color:#bdf0d1; }
    .badge.warn { background:#5b3d16; color:#ffd9a8; }
    .badge.bad { background:#5c2428; color:#ffb6bb; }
    .grid { display:grid; grid-template-columns:repeat(2, minmax(0,1fr)); gap:8px; }
    .grid button { width:100%; }
    .small { font-size:12px; }
    .divider { height:1px; background:#26313b; margin:10px 0; }
    .teleop { display:grid; grid-template-columns:repeat(3, 54px); grid-template-rows:repeat(3, 44px); gap:6px; justify-content:center; margin-top:8px; }
    .teleop button { padding:0; font-size:18px; font-weight:700; }
    .teleop .empty { visibility:hidden; }
    .hint { color:#8a98a8; font-size:12px; line-height:1.5; margin-top:8px; }
    @media (max-width: 980px) {
      .app { flex-direction:column; }
      .panel { width:auto; max-width:none; border-left:none; border-top:1px solid #26313b; }
      .content { flex-direction:column; }
      .canvas-wrap { min-height:52vh; }
    }
  </style>
</head>
<body>
  <div class="app">
    <div class="main">
      <div class="bar">
        <button onclick="sendAction({action:'set_mode', mode:0})">Idle</button>
        <button onclick="setSoloTeleop()">Teleop</button>
        <button onclick="setClusterDrive()">Formation Drive</button>
        <button onclick="sendAction({action:'set_mode', mode:3})">Follow</button>
        <button onclick="setFormation(0)">Column</button>
        <button onclick="setFormation(1)">Line</button>
        <button onclick="setFormation(2)">Circle</button>
        <button onclick="setFormation(3)">Triangle</button>
        <button onclick="sendAction({action:'toggle_avoidance'})">Avoid</button>
        <button onclick="sendAction({action:'set_control_mode', value:'body_orbit'})">Body</button>
        <button onclick="sendAction({action:'set_control_mode', value:'wheeltec_global'})">Global</button>
        <button onclick="sendAction({action:'return_home'})">Home</button>
        <button onclick="sendAction({action:'teleop', vx:0, wz:0})">Stop</button>
        <button id="goalToolButton" onclick="toggleGoalTool()">Set Goal</button>
        <button onclick="fitMapView()" title="Show the complete occupancy map">Fit Map</button>
        <button onclick="fitRobotView()" title="Follow the robot group">Fit Robots</button>
        <button onclick="zoomView(1.5)" title="Zoom in">+</button>
        <button onclick="zoomView(0.67)" title="Zoom out">-</button>
        <button onclick="sendAction({action:'clear_trails'})" title="Clear displayed trajectories">Clear Trails</button>
      </div>
      <div class="content">
        <div class="canvas-wrap"><canvas id="view"></canvas></div>
        <div class="panel">
          <div class="section">
            <div class="title">System</div>
            <div class="row"><span class="muted">Mode</span><span id="modeText">-</span></div>
            <div class="row"><span class="muted">Formation</span><span id="formationText">-</span></div>
            <div class="row"><span class="muted">Control</span><span id="controlModeText">-</span></div>
            <div class="row"><span class="muted">Avoidance</span><span id="avoidText">-</span></div>
            <div class="row"><span class="muted">Leader vx / wz</span><span id="leaderVelText">-</span></div>
            <div class="row"><span class="muted">Target</span><span id="targetText">-</span></div>
            <div class="row"><span class="muted">Nav goal</span><span id="navGoalText">-</span></div>
            <div class="row"><span class="muted">Navigation</span><span id="navStatusText">-</span></div>
            <div class="row"><span class="muted">Plans</span><span id="planText">-</span></div>
            <div class="row"><span class="muted">Action</span><span id="actionText">ready</span></div>
          </div>
          <div class="section">
            <div class="title">Teleop</div>
            <div class="teleop">
              <button class="empty"></button>
              <button onmousedown="holdTeleop(0.30,0)" ontouchstart="holdTeleop(0.30,0)" onmouseup="stopTeleop()" onmouseleave="stopTeleop()" ontouchend="stopTeleop()">W</button>
              <button class="empty"></button>
              <button onmousedown="holdTeleop(0,0.55)" ontouchstart="holdTeleop(0,0.55)" onmouseup="stopTeleop()" onmouseleave="stopTeleop()" ontouchend="stopTeleop()">A</button>
              <button onclick="sendAction({action:'teleop', vx:0, wz:0})">S</button>
              <button onmousedown="holdTeleop(0,-0.55)" ontouchstart="holdTeleop(0,-0.55)" onmouseup="stopTeleop()" onmouseleave="stopTeleop()" ontouchend="stopTeleop()">D</button>
              <button class="empty"></button>
              <button onmousedown="holdTeleop(-0.25,0)" ontouchstart="holdTeleop(-0.25,0)" onmouseup="stopTeleop()" onmouseleave="stopTeleop()" ontouchend="stopTeleop()">Back</button>
              <button class="empty"></button>
            </div>
            <div class="hint">Keyboard: W/S forward/back, A/D turn, Z/X/C/V modes, 1-4 formations, 0 home. Enable Set Goal, then click the map to send a car1 navigation goal. Mouse wheel zooms; drag the map while Set Goal is off.</div>
          </div>
          <div class="section">
            <div class="title">Robots</div>
            <div id="robotList"></div>
          </div>
        </div>
      </div>
    </div>
  </div>
  <script>
    var state = null;
    var canvas = document.getElementById('view');
    var ctx = canvas.getContext('2d');
    var teleopTimer = null;
    var pressedKeys = {};
    var goalToolEnabled = false;
    var mapCanvas = null;
    var loadedMapVersion = -1;
    var loadingMapVersion = -1;
    var viewMode = 'robots';
    var manualView = null;
    var dragState = null;
    var suppressNextClick = false;
    var clusterDriveEnabled = true;
    var clusterDriveModeSynced = false;
    var palette = {
      robot1: '#4da3ff',
      robot2: '#7ee081',
      robot3: '#ffb84d',
      target: '#c86cff',
      grid: '#1e2730',
      axis: '#5d6d7c'
    };

    function modeName(mode) {
      return {0:'IDLE', 1:'TELEOP', 2:'FORMATION', 3:'FOLLOW'}[mode] || ('MODE_' + mode);
    }

    function formationName(form) {
      return {0:'COLUMN', 1:'LINE', 2:'CIRCLE_SHOW', 3:'TRIANGLE'}[form] || ('FORM_' + form);
    }

    function clamp(v, lo, hi) {
      return Math.max(lo, Math.min(hi, v));
    }

    function resizeCanvas() {
      var rect = canvas.getBoundingClientRect();
      canvas.width = Math.max(1, Math.floor(rect.width * window.devicePixelRatio));
      canvas.height = Math.max(1, Math.floor(rect.height * window.devicePixelRatio));
      draw();
    }

    function posePoints() {
      var pts = [];
      if (!state) return pts;
      robotNames().forEach(function(name) {
        var robot = state.robots[name];
        if (!robot || !robot.pose) return;
        if (robot.pose.x === null || robot.pose.y === null) return;
        pts.push({x: robot.pose.x, y: robot.pose.y});
        if (robot.trail) {
          robot.trail.forEach(function(p) { pts.push({x: p.x, y: p.y}); });
        }
      });
      if (state.leader_cmd && state.leader_cmd.target_pose && state.leader_cmd.target_pose.visible) {
        pts.push({x: state.leader_cmd.target_pose.x, y: state.leader_cmd.target_pose.y});
      }
      if (state.assigned_goals) {
        Object.keys(state.assigned_goals).forEach(function(name) {
          var goal = state.assigned_goals[name];
          if (goal && goal.visible) pts.push({x:goal.x, y:goal.y});
        });
      }
      return pts;
    }

    function worldToScreen(x, y, view) {
      return {
        x: view.left + (x - view.minX) * view.scale,
        y: view.top + (view.maxY - y) * view.scale
      };
    }

    function screenToWorld(x, y, view) {
      return {
        x: view.minX + (x - view.left) / view.scale,
        y: view.maxY - (y - view.top) / view.scale
      };
    }

    function computeView() {
      var rect = canvas.getBoundingClientRect();
      var w = rect.width;
      var h = rect.height;
      if (viewMode === 'manual' && manualView) {
        var halfW = w / (2 * manualView.scale);
        var halfH = h / (2 * manualView.scale);
        return {
          minX:manualView.x-halfW, maxX:manualView.x+halfW,
          minY:manualView.y-halfH, maxY:manualView.y+halfH,
          scale:manualView.scale, left:0, top:0, width:w, height:h
        };
      }
      if (viewMode === 'map' && state && state.map && state.map.available) {
        return viewFromBounds(
          state.map.origin_x,
          state.map.origin_x + state.map.width * state.map.resolution,
          state.map.origin_y,
          state.map.origin_y + state.map.height * state.map.resolution,
          w, h, 0.5);
      }
      var pts = posePoints();
      if (!pts.length) {
        return { minX:-2, maxX:2, minY:-2, maxY:2, scale:1, left:0, top:0, width:w, height:h };
      }
      var minX = pts[0].x, maxX = pts[0].x, minY = pts[0].y, maxY = pts[0].y;
      pts.forEach(function(p) {
        minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
        minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
      });
      return viewFromBounds(minX, maxX, minY, maxY, w, h, 1.0);
    }

    function viewFromBounds(minX, maxX, minY, maxY, w, h, pad) {
      minX -= pad; maxX += pad; minY -= pad; maxY += pad;
      var dx = Math.max(maxX - minX, 1.0);
      var dy = Math.max(maxY - minY, 1.0);
      var scale = Math.min(w / dx, h / dy);
      var viewW = dx * scale;
      var viewH = dy * scale;
      var centerX = (minX + maxX) * 0.5;
      var centerY = (minY + maxY) * 0.5;
      minX = centerX - viewW / (2 * scale);
      maxX = centerX + viewW / (2 * scale);
      minY = centerY - viewH / (2 * scale);
      maxY = centerY + viewH / (2 * scale);
      return {
        minX: minX, maxX: maxX, minY: minY, maxY: maxY,
        scale: scale, left: (w - viewW) * 0.5, top: (h - viewH) * 0.5,
        width: viewW, height: viewH
      };
    }

    function drawGrid(view) {
      var step = 0.5;
      var startX = Math.floor(view.minX / step) * step;
      var endX = Math.ceil(view.maxX / step) * step;
      var startY = Math.floor(view.minY / step) * step;
      var endY = Math.ceil(view.maxY / step) * step;

      ctx.lineWidth = 1;
      for (var x = startX; x <= endX; x += step) {
        var p1 = worldToScreen(x, view.minY, view);
        var p2 = worldToScreen(x, view.maxY, view);
        ctx.strokeStyle = Math.abs(x) < 1e-6 ? palette.axis : palette.grid;
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
      }
      for (var y = startY; y <= endY; y += step) {
        var q1 = worldToScreen(view.minX, y, view);
        var q2 = worldToScreen(view.maxX, y, view);
        ctx.strokeStyle = Math.abs(y) < 1e-6 ? palette.axis : palette.grid;
        ctx.beginPath();
        ctx.moveTo(q1.x, q1.y);
        ctx.lineTo(q2.x, q2.y);
        ctx.stroke();
      }
    }

    function drawMap(view) {
      if (!mapCanvas || !state || !state.map || !state.map.available) return;
      var info = state.map;
      // OccupancyGrid map origins in this project have zero yaw. Refuse to
      // silently misalign clicks if a rotated map is loaded later.
      if (Math.abs(info.origin_yaw || 0) > 1e-6) return;
      var mapMaxX = info.origin_x + info.width * info.resolution;
      var mapMaxY = info.origin_y + info.height * info.resolution;
      var visibleMinX = Math.max(view.minX, info.origin_x);
      var visibleMaxX = Math.min(view.maxX, mapMaxX);
      var visibleMinY = Math.max(view.minY, info.origin_y);
      var visibleMaxY = Math.min(view.maxY, mapMaxY);
      if (visibleMinX >= visibleMaxX || visibleMinY >= visibleMaxY) return;
      var sourceX = (visibleMinX - info.origin_x) / info.resolution;
      var sourceY = info.height - (visibleMaxY - info.origin_y) / info.resolution;
      var sourceW = (visibleMaxX - visibleMinX) / info.resolution;
      var sourceH = (visibleMaxY - visibleMinY) / info.resolution;
      var topLeft = worldToScreen(visibleMinX, visibleMaxY, view);
      var bottomRight = worldToScreen(visibleMaxX, visibleMinY, view);
      ctx.save();
      ctx.globalAlpha = 0.82;
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(mapCanvas, sourceX, sourceY, sourceW, sourceH,
                    topLeft.x, topLeft.y,
                    bottomRight.x - topLeft.x, bottomRight.y - topLeft.y);
      ctx.restore();
    }

    function drawPath(points, color, width, view) {
      if (!points || points.length < 2) return;
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.beginPath();
      points.forEach(function(point, index) {
        var p = worldToScreen(point.x, point.y, view);
        if (index === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
      });
      ctx.stroke();
    }

    function drawTrail(robot, color, view) {
      if (!robot.trail || !robot.trail.length) return;
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (var i = 0; i < robot.trail.length; i++) {
        var p = worldToScreen(robot.trail[i].x, robot.trail[i].y, view);
        if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
      }
      ctx.stroke();
    }

    function drawRobot(robot, color, view, label) {
      if (!robot || !robot.pose) return;
      var pose = robot.pose;
      if (pose.x === null || pose.y === null) return;
      var p = worldToScreen(pose.x, pose.y, view);
      var r = 10;
      ctx.fillStyle = color;
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      var hx = p.x + Math.cos(pose.yaw) * r * 1.8;
      var hy = p.y - Math.sin(pose.yaw) * r * 1.8;
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(p.x, p.y);
      ctx.lineTo(hx, hy);
      ctx.stroke();

      ctx.fillStyle = '#dce6f0';
      ctx.font = '12px Arial';
      ctx.fillText(label, p.x + r + 4, p.y - r - 2);
    }

    function drawTarget(view) {
      if (!state || !state.leader_cmd || !state.leader_cmd.target_pose || !state.leader_cmd.target_pose.visible) return;
      var t = state.leader_cmd.target_pose;
      var p = worldToScreen(t.x, t.y, view);
      ctx.strokeStyle = palette.target;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(p.x - 8, p.y);
      ctx.lineTo(p.x + 8, p.y);
      ctx.moveTo(p.x, p.y - 8);
      ctx.lineTo(p.x, p.y + 8);
      ctx.stroke();
      ctx.fillStyle = palette.target;
      ctx.fillText('T', p.x + 10, p.y - 10);
    }

    function drawNavGoal(view) {
      if (!state || !state.nav_goal || !state.nav_goal.visible) return;
      var g = state.nav_goal;
      var p = worldToScreen(g.x, g.y, view);
      ctx.strokeStyle = '#ff5ca8';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 9, 0, Math.PI * 2);
      ctx.moveTo(p.x - 14, p.y);
      ctx.lineTo(p.x + 14, p.y);
      ctx.moveTo(p.x, p.y - 14);
      ctx.lineTo(p.x, p.y + 14);
      ctx.stroke();
      ctx.fillStyle = '#ff5ca8';
      ctx.fillText('G', p.x + 12, p.y - 12);
    }

    function drawAssignedGoal(name, color, view, label) {
      if (!state || !state.assigned_goals) return;
      var goal = state.assigned_goals[name];
      if (!goal || !goal.visible) return;
      var p = worldToScreen(goal.x, goal.y, view);
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(p.x - 7, p.y - 7);
      ctx.lineTo(p.x + 7, p.y + 7);
      ctx.moveTo(p.x - 7, p.y + 7);
      ctx.lineTo(p.x + 7, p.y - 7);
      ctx.stroke();
      ctx.fillStyle = color;
      ctx.fillText(label, p.x + 9, p.y - 8);
    }

    function draw() {
      var rect = canvas.getBoundingClientRect();
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.save();
      ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
      ctx.fillStyle = '#0f1419';
      ctx.fillRect(0, 0, rect.width, rect.height);

      if (!state) {
        ctx.fillStyle = '#8a98a8';
        ctx.font = '16px Arial';
        ctx.fillText('Waiting for ROS data...', 20, 30);
        ctx.restore();
        return;
      }

      var view = computeView();
      drawMap(view);
      drawGrid(view);
      drawPath(state.global_plan, '#27c2ff', 3, view);
      drawPath(state.local_plan, '#ff5ca8', 2, view);
      robotNames().forEach(function(name) {
        drawTrail(state.robots[name], palette[name], view);
      });
      drawTarget(view);
      drawNavGoal(view);
      drawAssignedGoal('robot2', palette.robot2, view, 'G2');
      drawAssignedGoal('robot3', palette.robot3, view, 'G3');
      drawRobot(state.robots.robot1, palette.robot1, view, '1');
      drawRobot(state.robots.robot2, palette.robot2, view, '2');
      if (state.robots.robot3) drawRobot(state.robots.robot3, palette.robot3, view, '3');

      ctx.fillStyle = '#8a98a8';
      ctx.font = '12px Arial';
      ctx.fillText('Frame: ' + state.frame, 12, rect.height - 12);
      ctx.restore();
    }

    function setText(id, text) {
      document.getElementById(id).textContent = text;
    }

    function robotStatusLine(name, robot) {
      if (!robot) return '<div class="robot"><div class="status-line"><span>' + name + '</span><span class="badge bad">offline</span></div></div>';
      var pose = robot.pose || {};
      var status = robot.status || {};
      var stale = robot.age !== null && robot.age !== undefined && robot.age > state.pose_timeout;
      var badgeClass = robot.connected ? (stale ? 'warn' : 'ok') : 'bad';
      var source = robot.source || '-';
      return '' +
        '<div class="robot">' +
          '<div class="status-line"><span>' + name + '</span><span class="badge ' + badgeClass + '">' + (robot.connected ? (stale ? 'stale' : 'live') : 'lost') + '</span></div>' +
          '<div class="small muted">x ' + fmt(pose.x) + '  y ' + fmt(pose.y) + '  yaw ' + fmt(pose.yaw) + '</div>' +
          '<div class="small muted">src ' + source + '  age ' + fmt(robot.age) + 's</div>' +
          '<div class="small muted">v ' + fmt(robot.vx) + '  w ' + fmt(robot.wz) + '</div>' +
          '<div class="small muted">err ' + fmt(status.error_dist) + '  yaw ' + fmt(status.error_yaw) + '</div>' +
        '</div>';
    }

    function fmt(v) {
      if (v === null || v === undefined || isNaN(v)) return '-';
      return Number(v).toFixed(2);
    }

    function updatePanel() {
      if (!state) return;
      setText('modeText', modeName(state.leader_cmd.mode));
      setText('formationText', formationName(state.leader_cmd.formation));
      setText('controlModeText', state.control_mode || '-');
      setText('avoidText', state.avoidance_enabled ? 'on' : 'off');
      setText('leaderVelText', fmt(state.leader_cmd.leader_vx) + ' / ' + fmt(state.leader_cmd.leader_vyaw));
      if (state.leader_cmd.target_pose && state.leader_cmd.target_pose.visible) {
        setText('targetText', fmt(state.leader_cmd.target_pose.x) + ', ' + fmt(state.leader_cmd.target_pose.y) + ', ' + fmt(state.leader_cmd.target_pose.yaw));
      } else {
        setText('targetText', '-');
      }
      if (state.nav_goal && state.nav_goal.visible) {
        setText('navGoalText', fmt(state.nav_goal.x) + ', ' + fmt(state.nav_goal.y) + ', ' + fmt(state.nav_goal.yaw));
      } else {
        setText('navGoalText', '-');
      }
      var nav = state.navigation || {};
      setText('navStatusText', nav.label || 'unavailable');
      setText('planText', (state.global_plan || []).length + ' global / ' +
              (state.local_plan || []).length + ' local');
      document.getElementById('robotList').innerHTML =
        robotStatusLine('Robot 1', state.robots.robot1) +
        robotStatusLine('Robot 2', state.robots.robot2) +
        (state.robots.robot3 ? robotStatusLine('Robot 3', state.robots.robot3) : '');
    }

    function robotNames() {
      if (!state || !state.robots) return ['robot1','robot2','robot3'];
      return ['robot1','robot2','robot3'].filter(function(name) { return !!state.robots[name]; });
    }

    function sendAction(payload, refreshAfter) {
      if (refreshAfter === undefined) refreshAfter = true;
      fetch('/api/action', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(payload)
      }).then(function(resp) { return resp.json(); })
        .then(function(data) {
          if (data) setText('actionText', data.message || (data.ok ? 'ok' : 'action failed'));
          if (data && data.ok === false) console.warn(data.message || 'action failed');
          if (refreshAfter) refresh();
        }).catch(function(err) {
          console.error(err);
        });
    }

    function setClusterDrive() {
      clusterDriveEnabled = true;
      var form = state && state.leader_cmd ? state.leader_cmd.formation : 0;
      sendAction({action:'set_mode', mode:2, formation:form});
      clusterDriveModeSynced = true;
    }

    function setSoloTeleop() {
      clusterDriveEnabled = false;
      clusterDriveModeSynced = false;
      sendAction({action:'set_mode', mode:1});
    }

    function setFormation(form) {
      clusterDriveEnabled = true;
      clusterDriveModeSynced = true;
      sendAction({action:'set_formation', formation:form});
    }

    function toggleGoalTool() {
      goalToolEnabled = !goalToolEnabled;
      var btn = document.getElementById('goalToolButton');
      btn.style.background = goalToolEnabled ? '#4a2540' : '#1c2630';
    }

    function fitMapView() {
      viewMode = 'map';
      manualView = null;
      draw();
    }

    function fitRobotView() {
      viewMode = 'robots';
      manualView = null;
      draw();
    }

    function zoomView(factor, anchorX, anchorY) {
      var rect = canvas.getBoundingClientRect();
      var view = computeView();
      if (anchorX === undefined) anchorX = rect.width * 0.5;
      if (anchorY === undefined) anchorY = rect.height * 0.5;
      var anchor = screenToWorld(anchorX, anchorY, view);
      var newScale = clamp(view.scale * factor, 2.0, 1000.0);
      manualView = {
        scale:newScale,
        x:anchor.x - (anchorX - rect.width * 0.5) / newScale,
        y:anchor.y + (anchorY - rect.height * 0.5) / newScale
      };
      viewMode = 'manual';
      draw();
    }

    function beginPan(e) {
      if (e.button === 0 && goalToolEnabled) return;
      if (e.button !== 0 && e.button !== 1 && e.button !== 2) return;
      e.preventDefault();
      var rect = canvas.getBoundingClientRect();
      var view = computeView();
      manualView = {
        scale:view.scale,
        x:(view.minX + view.maxX) * 0.5,
        y:(view.minY + view.maxY) * 0.5
      };
      dragState = {
        x:e.clientX, y:e.clientY,
        centerX:manualView.x, centerY:manualView.y,
        moved:false, button:e.button
      };
      viewMode = 'manual';
    }

    function continuePan(e) {
      if (!dragState) return;
      var dx = e.clientX - dragState.x;
      var dy = e.clientY - dragState.y;
      if (Math.abs(dx) + Math.abs(dy) > 3) dragState.moved = true;
      manualView.x = dragState.centerX - dx / manualView.scale;
      manualView.y = dragState.centerY + dy / manualView.scale;
      draw();
    }

    function endPan() {
      if (!dragState) return;
      if (dragState.moved && dragState.button === 0) suppressNextClick = true;
      dragState = null;
    }

    function canvasClick(e) {
      if (suppressNextClick) {
        suppressNextClick = false;
        return;
      }
      if (!goalToolEnabled || !state) return;
      // A missed keyup can leave the browser teleop timer publishing forever.
      // Stop that local stream before handing velocity control to move_base.
      if (teleopTimer) {
        clearInterval(teleopTimer);
        teleopTimer = null;
      }
      pressedKeys = {};
      var rect = canvas.getBoundingClientRect();
      var view = computeView();
      var p = screenToWorld(e.clientX - rect.left, e.clientY - rect.top, view);
      var yaw = 0.0;
      if (state.robots.robot1 && state.robots.robot1.pose && state.robots.robot1.pose.yaw !== null) {
        yaw = state.robots.robot1.pose.yaw;
      }
      sendAction({action:'nav_goal', x:p.x, y:p.y, yaw:yaw});
    }

    function holdTeleop(vx, wz) {
      if (typeof event !== 'undefined' && event && event.preventDefault) event.preventDefault();
      if (clusterDriveEnabled) {
        var mode = state && state.leader_cmd ? state.leader_cmd.mode : 0;
        if (!clusterDriveModeSynced || mode !== 2) {
          var form = state && state.leader_cmd ? state.leader_cmd.formation : 0;
          sendAction({action:'set_mode', mode:2, formation:form}, false);
          clusterDriveModeSynced = true;
        }
      }
      sendAction({action:'teleop', vx:vx, wz:wz}, false);
      if (teleopTimer) clearInterval(teleopTimer);
      teleopTimer = setInterval(function() {
        sendAction({action:'teleop', vx:vx, wz:wz}, false);
      }, 100);
    }

    function stopTeleop() {
      if (teleopTimer) {
        clearInterval(teleopTimer);
        teleopTimer = null;
      }
      sendAction({action:'teleop', vx:0, wz:0}, false);
    }

    function handleShortcut(key, down) {
      key = key.toLowerCase();
      if (down && pressedKeys[key]) return;
      pressedKeys[key] = down;
      if (!down) {
        if ('wasd'.indexOf(key) !== -1) stopTeleop();
        return;
      }
      if (key === 'w') holdTeleop(0.30, 0);
      else if (key === 's') holdTeleop(-0.25, 0);
      else if (key === 'a') holdTeleop(0, 0.55);
      else if (key === 'd') holdTeleop(0, -0.55);
      else if (key === 'z') { clusterDriveModeSynced = false; sendAction({action:'set_mode', mode:0}); }
      else if (key === 'x') setSoloTeleop();
      else if (key === 'c') setClusterDrive();
      else if (key === 'v') { clusterDriveEnabled = false; clusterDriveModeSynced = false; sendAction({action:'set_mode', mode:3}); }
      else if (key === '1') setFormation(0);
      else if (key === '2') setFormation(1);
      else if (key === '3') setFormation(2);
      else if (key === '4') setFormation(3);
      else if (key === '0') sendAction({action:'return_home'});
    }

    function refresh() {
      fetch('/api/state')
        .then(function(resp) { return resp.json(); })
        .then(function(data) {
          state = data;
          loadMapIfNeeded();
          updatePanel();
          draw();
        })
        .catch(function() {});
    }

    function loadMapIfNeeded() {
      if (!state || !state.map || !state.map.available) return;
      var version = state.map.version;
      if (version === loadedMapVersion || version === loadingMapVersion) return;
      loadingMapVersion = version;
      fetch('/api/map?version=' + encodeURIComponent(version))
        .then(function(resp) {
          if (!resp.ok) throw new Error('map unavailable');
          return resp.arrayBuffer();
        })
        .then(function(buffer) {
          if (!state || !state.map || state.map.version !== version) return;
          var info = state.map;
          var values = new Uint8Array(buffer);
          if (values.length !== info.width * info.height) {
            throw new Error('map size mismatch');
          }
          var offscreen = document.createElement('canvas');
          offscreen.width = info.width;
          offscreen.height = info.height;
          var offctx = offscreen.getContext('2d');
          var image = offctx.createImageData(info.width, info.height);
          for (var row = 0; row < info.height; row++) {
            var dstRow = info.height - 1 - row;
            for (var col = 0; col < info.width; col++) {
              var value = values[row * info.width + col];
              var shade;
              if (value === 255) shade = 72;
              else if (value >= 65) shade = 18;
              else if (value === 0) shade = 220;
              else shade = Math.max(35, 220 - Math.round(value * 1.85));
              var index = (dstRow * info.width + col) * 4;
              image.data[index] = shade;
              image.data[index + 1] = shade;
              image.data[index + 2] = shade;
              image.data[index + 3] = 255;
            }
          }
          offctx.putImageData(image, 0, 0);
          mapCanvas = offscreen;
          loadedMapVersion = version;
          draw();
        })
        .catch(function(err) { console.error(err); })
        .then(function() { loadingMapVersion = -1; });
    }

    window.addEventListener('resize', resizeCanvas);
    canvas.addEventListener('click', canvasClick);
    canvas.addEventListener('mousedown', beginPan);
    window.addEventListener('mousemove', continuePan);
    window.addEventListener('mouseup', endPan);
    canvas.addEventListener('contextmenu', function(e) { e.preventDefault(); });
    canvas.addEventListener('wheel', function(e) {
      e.preventDefault();
      var rect = canvas.getBoundingClientRect();
      zoomView(e.deltaY < 0 ? 1.2 : 0.83,
               e.clientX - rect.left, e.clientY - rect.top);
    }, {passive:false});
    window.addEventListener('keydown', function(e) {
      if (e.target && ['INPUT','TEXTAREA','SELECT'].indexOf(e.target.tagName) !== -1) return;
      handleShortcut(e.key, true);
    });
    window.addEventListener('keyup', function(e) { handleShortcut(e.key, false); });
    setInterval(refresh, 250);
    resizeCanvas();
    refresh();
  </script>
</body>
</html>
"""


def _clamp(value, low, high):
    return max(low, min(high, value))


def _quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _pose_to_dict(x, y, yaw, source, stamp, visible=True):
    return {
        'x': x,
        'y': y,
        'yaw': yaw,
        'source': source,
        'stamp': float(stamp) if stamp is not None else 0.0,
        'visible': visible
    }


def _to_utf8_bytes(value):
    if isinstance(value, bytes):
        return value
    if isinstance(value, unicode):
        return value.encode('utf-8')
    return str(value).encode('utf-8')


class _ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class _VisualizerRequestHandler(BaseHTTPRequestHandler):
    server_version = 'ClusterWebVisualizer/1.0'

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path in ('/', '/index.html'):
            self._send_text(INDEX_HTML, 'text/html; charset=utf-8')
            return
        if path == '/api/state':
            self._send_json(self.server.app.build_state())
            return
        if path == '/api/map':
            payload = self.server.app.map_payload()
            if payload is None:
                self.send_error(503, 'Map unavailable')
            else:
                self._send_binary(payload, 'application/octet-stream')
            return
        self.send_error(404, 'Not Found')

    def do_POST(self):
        path = self.path.split('?', 1)[0]
        if path != '/api/action':
            self.send_error(404, 'Not Found')
            return

        length = int(self.headers.get('Content-Length', '0') or '0')
        payload = {}
        if length > 0:
            raw = self.rfile.read(length)
            try:
                payload = json.loads(raw.decode('utf-8'))
            except Exception as exc:
                self._send_json({'ok': False, 'message': 'Invalid JSON: %s' % exc}, status=400)
                return

        result = self.server.app.handle_action(payload)
        self._send_json(result)

    def log_message(self, fmt, *args):
        rospy.loginfo('%s - %s', self.address_string(), fmt % args)

    def _send_text(self, body, content_type):
        encoded = _to_utf8_bytes(body)
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(encoded)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(encoded)

    def _send_json(self, obj, status=200):
        encoded = json.dumps(obj).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(encoded)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(encoded)

    def _send_binary(self, body, content_type):
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)


class ClusterWebVisualizer(object):
    def __init__(self):
        self.host = rospy.get_param('~host', '0.0.0.0')
        self.port = int(rospy.get_param('~port', 8080))
        self.frame = rospy.get_param('~frame', 'map')
        self.trail_length = int(rospy.get_param('~trail_length', 180))
        self.pose_timeout = float(rospy.get_param('~pose_timeout', 1.5))
        self.enable_tf = bool(rospy.get_param('~enable_tf', True))
        self.show_robot3 = bool(rospy.get_param('~show_robot3', True))
        self.max_teleop_vx = float(rospy.get_param('~max_teleop_vx', 0.35))
        self.max_teleop_wz = float(rospy.get_param('~max_teleop_wz', 0.65))
        self.teleop_topic = rospy.get_param('~teleop_topic', '/robot1/teleop_vel')
        self.nav_goal_topic = rospy.get_param('~nav_goal_topic',
                                              '/robot1/move_base_simple/goal')
        self.require_follower_localization = bool(rospy.get_param(
            '~require_follower_localization', True))

        self.state_lock = threading.RLock()
        self.latest_odom = {}
        self.latest_status = {'robot2': None, 'robot3': None}
        self.map_bytes = None
        self.map_info = None
        self.map_version = 0
        self.global_plan = []
        self.local_plan = []
        self.assigned_goals = {'robot2': None, 'robot3': None}
        self.navigation = {'status': None, 'label': 'waiting for move_base'}
        self.leader_cmd = None
        self.control_mode = 'body_orbit'
        self.avoidance_enabled = True
        self.control_mode_time = 0.0
        self.avoidance_time = 0.0
        self.last_teleop = {'vx': 0.0, 'wz': 0.0, 'stamp': 0.0}
        self.last_nav_goal = {
            'x': None,
            'y': None,
            'yaw': None,
            'source': 'none',
            'stamp': 0.0,
            'visible': False
        }
        self.trails = {
            'robot1': [],
            'robot2': [],
            'robot3': [],
        }

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.odom_subs = {
            'robot1': rospy.Subscriber('/robot1/odom', Odometry,
                                        self._make_odom_callback('robot1'), queue_size=10),
            'robot2': rospy.Subscriber('/robot2/odom', Odometry,
                                        self._make_odom_callback('robot2'), queue_size=10),
            'robot3': rospy.Subscriber('/robot3/odom', Odometry,
                                        self._make_odom_callback('robot3'), queue_size=10),
        }
        self.leader_cmd_sub = rospy.Subscriber('/robot1/leader_cmd', LeaderCmd,
                                               self._leader_cmd_callback, queue_size=10)
        self.status_subs = {
            'robot2': rospy.Subscriber('/robot2/follower_status', FollowerStatus,
                                        self._make_status_callback('robot2'), queue_size=10),
            'robot3': rospy.Subscriber('/robot3/follower_status', FollowerStatus,
                                        self._make_status_callback('robot3'), queue_size=10),
        }
        self.control_mode_sub = rospy.Subscriber('/robot2/follower_control_mode', String,
                                                 self._control_mode_callback, queue_size=1)
        self.avoidance_sub = rospy.Subscriber('/robot2/avoidance_enabled', Bool,
                                              self._avoidance_callback, queue_size=1)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid,
                                        self._map_callback, queue_size=1)
        self.global_plan_sub = rospy.Subscriber(
            '/robot1/move_base/GlobalPlanner/plan', Path,
            self._make_path_callback('global'), queue_size=1)
        self.local_plan_sub = rospy.Subscriber(
            '/robot1/move_base/DWAPlannerROS/local_plan', Path,
            self._make_path_callback('local'), queue_size=1)
        self.nav_status_sub = rospy.Subscriber(
            '/robot1/move_base/status', GoalStatusArray,
            self._nav_status_callback, queue_size=1)
        self.assigned_goal_subs = {
            'robot2': rospy.Subscriber(
                '/robot2/assigned_goal', PoseStamped,
                self._make_assigned_goal_callback('robot2'), queue_size=1),
            'robot3': rospy.Subscriber(
                '/robot3/assigned_goal', PoseStamped,
                self._make_assigned_goal_callback('robot3'), queue_size=1),
        }

        self.return_home_pub = rospy.Publisher('/robot1/return_home', Bool, queue_size=1)
        self.teleop_pub = rospy.Publisher(self.teleop_topic, Twist, queue_size=1)
        self.nav_goal_pub = rospy.Publisher(self.nav_goal_topic, PoseStamped,
                                            queue_size=1)
        self.nav_cancel_pub = rospy.Publisher('/robot1/move_base/cancel', GoalID,
                                              queue_size=1)
        self.control_mode_pub = rospy.Publisher('/robot2/follower_control_mode', String, queue_size=1, latch=True)
        self.avoidance_pub = rospy.Publisher('/robot2/avoidance_enabled', Bool, queue_size=1, latch=True)
        self.set_mode_srv_name = '/robot1/set_mode'
        self.set_formation_srv_name = '/robot1/set_formation'

        rospy.on_shutdown(self.shutdown)
        self.http_server = _ThreadedHTTPServer((self.host, self.port), _VisualizerRequestHandler)
        self.http_server.app = self
        self.http_thread = threading.Thread(target=self.http_server.serve_forever)
        self.http_thread.daemon = True
        self.http_thread.start()
        rospy.loginfo('Web visualizer listening on http://%s:%d', self.host, self.port)

    def _map_callback(self, msg):
        payload = bytearray(len(msg.data))
        for index, value in enumerate(msg.data):
            payload[index] = value if value >= 0 else 255
        origin = msg.info.origin
        info = {
            'available': True,
            'width': int(msg.info.width),
            'height': int(msg.info.height),
            'resolution': float(msg.info.resolution),
            'origin_x': float(origin.position.x),
            'origin_y': float(origin.position.y),
            'origin_yaw': float(_quat_to_yaw(origin.orientation)),
        }
        with self.state_lock:
            self.map_bytes = bytes(payload)
            self.map_version += 1
            info['version'] = self.map_version
            self.map_info = info

    def map_payload(self):
        with self.state_lock:
            return self.map_bytes

    def _make_path_callback(self, path_kind):
        def _cb(msg):
            points = []
            frame = msg.header.frame_id or self.frame
            tx = 0.0
            ty = 0.0
            yaw = 0.0
            if frame != self.frame:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.frame, frame, rospy.Time(0), rospy.Duration(0.05))
                    tx = transform.transform.translation.x
                    ty = transform.transform.translation.y
                    yaw = _quat_to_yaw(transform.transform.rotation)
                except Exception:
                    return
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                points.append({
                    'x': tx + cos_yaw * x - sin_yaw * y,
                    'y': ty + sin_yaw * x + cos_yaw * y,
                })
            with self.state_lock:
                if path_kind == 'global':
                    self.global_plan = points
                else:
                    self.local_plan = points
        return _cb

    def _nav_status_callback(self, msg):
        labels = {
            0: 'PENDING', 1: 'ACTIVE', 2: 'PREEMPTED', 3: 'SUCCEEDED',
            4: 'ABORTED', 5: 'REJECTED', 6: 'PREEMPTING', 7: 'RECALLING',
            8: 'RECALLED', 9: 'LOST'
        }
        if not msg.status_list:
            navigation = {'status': None, 'label': 'no goal'}
        else:
            latest = msg.status_list[-1]
            label = labels.get(int(latest.status), 'STATUS_%d' % latest.status)
            if latest.text:
                label += ': ' + latest.text
            navigation = {'status': int(latest.status), 'label': label}
        with self.state_lock:
            self.navigation = navigation

    def _make_assigned_goal_callback(self, robot):
        def _cb(msg):
            if msg.header.frame_id != self.frame:
                return
            goal = _pose_to_dict(
                msg.pose.position.x, msg.pose.position.y,
                _quat_to_yaw(msg.pose.orientation), 'assigned_goal',
                msg.header.stamp.to_sec() if msg.header.stamp else
                rospy.Time.now().to_sec(), True)
            with self.state_lock:
                self.assigned_goals[robot] = goal
        return _cb

    def _make_odom_callback(self, name):
        def _cb(msg):
            pose = self._pose_from_odom(msg)
            with self.state_lock:
                self.latest_odom[name] = {
                    'pose': pose,
                    'vx': msg.twist.twist.linear.x,
                    'wz': msg.twist.twist.angular.z,
                    'stamp': msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
                }
        return _cb

    def _make_status_callback(self, name):
        def _cb(msg):
            with self.state_lock:
                self.latest_status[name] = {
                    'state': int(msg.state),
                    'error_x': float(msg.error_x),
                    'error_y': float(msg.error_y),
                    'error_yaw': float(msg.error_yaw),
                    'error_dist': float(msg.error_dist),
                    'leader_visible': bool(msg.leader_visible),
                    'stamp': msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
                }
        return _cb

    def _leader_cmd_callback(self, msg):
        with self.state_lock:
            self.leader_cmd = {
                'mode': int(msg.mode),
                'formation': int(msg.formation),
                'offset_x': float(msg.offset_x),
                'offset_y': float(msg.offset_y),
                'offset_yaw': float(msg.offset_yaw),
                'leader_vx': float(msg.leader_vx),
                'leader_vyaw': float(msg.leader_vyaw),
                'speed_limit': float(msg.speed_limit),
                'target_pose': self._pose_to_dict_from_pose(msg.target_pose)
            }

    def _control_mode_callback(self, msg):
        with self.state_lock:
            self.control_mode = msg.data
            self.control_mode_time = rospy.Time.now().to_sec()

    def _avoidance_callback(self, msg):
        with self.state_lock:
            self.avoidance_enabled = bool(msg.data)
            self.avoidance_time = rospy.Time.now().to_sec()

    def _pose_from_odom(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        return _pose_to_dict(position.x, position.y, _quat_to_yaw(orientation),
                             'odom', msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec(),
                             True)

    def _pose_to_dict_from_pose(self, pose_msg):
        position = pose_msg.position
        orientation = pose_msg.orientation
        x = position.x
        y = position.y
        if abs(x) < 1e-9 and abs(y) < 1e-9 and abs(orientation.x) < 1e-9 and abs(orientation.y) < 1e-9 and abs(orientation.z) < 1e-9 and abs(orientation.w) < 1e-9:
            return {'x': None, 'y': None, 'yaw': None, 'source': 'none', 'stamp': 0.0, 'visible': False}
        return _pose_to_dict(x, y, _quat_to_yaw(orientation), 'leader_cmd', rospy.Time.now().to_sec(), True)

    def _lookup_tf_pose(self, robot):
        if not self.enable_tf:
            return None
        frame = robot + '/base_link'
        try:
            tf = self.tf_buffer.lookup_transform(self.frame, frame, rospy.Time(0), rospy.Duration(0.05))
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        stamp = tf.header.stamp.to_sec() if tf.header.stamp else rospy.Time.now().to_sec()
        if stamp:
            age = rospy.Time.now().to_sec() - stamp
            if age > self.pose_timeout or age < -0.2:
                return None
        return _pose_to_dict(t.x, t.y, _quat_to_yaw(q), 'tf', stamp, True)

    def _active_pose(self, robot):
        pose = self._lookup_tf_pose(robot)
        if pose is not None:
            return pose
        item = self.latest_odom.get(robot)
        if item is None:
            return {'x': None, 'y': None, 'yaw': None, 'source': 'none', 'stamp': 0.0, 'visible': False}
        if self.enable_tf and self.frame == 'map':
            return {
                'x': None,
                'y': None,
                'yaw': None,
                'source': 'tf_missing',
                'stamp': item['stamp'],
                'visible': False
            }
        pose = dict(item['pose'])
        pose['source'] = 'odom'
        return pose

    def _refresh_trails(self, robot, pose):
        if pose is None or pose.get('x') is None or pose.get('y') is None:
            return
        trail = self.trails[robot]
        if trail:
            last = trail[-1]
            if abs(last['x'] - pose['x']) < 1e-4 and abs(last['y'] - pose['y']) < 1e-4:
                return
        trail.append({'x': pose['x'], 'y': pose['y']})
        if len(trail) > self.trail_length:
            del trail[:len(trail) - self.trail_length]

    def build_state(self):
        with self.state_lock:
            robots = {}
            robot_names = ['robot1', 'robot2']
            if self.show_robot3:
                robot_names.append('robot3')
            for name in robot_names:
                pose = self._active_pose(name)
                self._refresh_trails(name, pose)
                odom = self.latest_odom.get(name)
                status = self.latest_status.get(name)
                stamp = pose.get('stamp', 0.0)
                age = max(0.0, rospy.Time.now().to_sec() - stamp) if stamp else None
                robots[name] = {
                    'pose': pose,
                    'trail': list(self.trails[name]),
                    'vx': odom['vx'] if odom else 0.0,
                    'wz': odom['wz'] if odom else 0.0,
                    'age': age,
                    'connected': pose.get('x') is not None,
                    'source': pose.get('source', 'none'),
                    'status': status,
                }

            leader_cmd = self.leader_cmd or {
                'mode': 0,
                'formation': 0,
                'offset_x': 0.0,
                'offset_y': 0.0,
                'offset_yaw': 0.0,
                'leader_vx': 0.0,
                'leader_vyaw': 0.0,
                'speed_limit': 0.0,
                'target_pose': {'x': None, 'y': None, 'yaw': None, 'source': 'none', 'stamp': 0.0, 'visible': False}
            }

            return {
                'frame': self.frame,
                'stamp': rospy.Time.now().to_sec(),
                'robots': robots,
                'leader_cmd': leader_cmd,
                'control_mode': self.control_mode,
                'avoidance_enabled': self.avoidance_enabled,
                'pose_timeout': self.pose_timeout,
                'teleop': dict(self.last_teleop),
                'nav_goal': dict(self.last_nav_goal),
                'map': dict(self.map_info) if self.map_info is not None else {
                    'available': False, 'version': 0
                },
                'global_plan': list(self.global_plan),
                'local_plan': list(self.local_plan),
                'navigation': dict(self.navigation),
                'assigned_goals': {
                    name: (dict(goal) if goal is not None else None)
                    for name, goal in self.assigned_goals.items()
                },
            }

    def handle_action(self, payload):
        action = payload.get('action')
        try:
            if action == 'set_mode':
                mode = int(payload.get('mode', 0))
                formation = int(payload.get('formation', 0))
                if mode != 2:
                    self.nav_cancel_pub.publish(GoalID())
                return self._call_set_mode(mode, formation, 0.0, 0.0, 0.0)
            if action == 'set_formation':
                formation = int(payload.get('formation', 0))
                return self._call_set_formation(formation)
            if action == 'return_home':
                self.return_home_pub.publish(Bool(data=True))
                return {'ok': True, 'message': 'return_home published'}
            if action == 'teleop':
                vx = _clamp(float(payload.get('vx', 0.0)),
                            -self.max_teleop_vx, self.max_teleop_vx)
                wz = _clamp(float(payload.get('wz', 0.0)),
                            -self.max_teleop_wz, self.max_teleop_wz)
                cmd = Twist()
                cmd.linear.x = vx
                cmd.angular.z = wz
                if abs(vx) > 0.01 or abs(wz) > 0.01:
                    self.nav_cancel_pub.publish(GoalID())
                self.teleop_pub.publish(cmd)
                with self.state_lock:
                    self.last_teleop = {
                        'vx': vx,
                        'wz': wz,
                        'stamp': rospy.Time.now().to_sec()
                    }
                return {'ok': True, 'message': 'teleop published'}
            if action == 'nav_goal':
                x = float(payload.get('x', 0.0))
                y = float(payload.get('y', 0.0))
                yaw = float(payload.get('yaw', 0.0))

                if self.require_follower_localization:
                    missing = [name for name in ('robot2', 'robot3')
                               if self._lookup_tf_pose(name) is None]
                    if missing:
                        return {
                            'ok': False,
                            'message': 'navigation rejected: localization lost for ' +
                                       ', '.join(missing)
                        }

                # Serialize the handoff from browser teleop to move_base.  The
                # zero is ordered after earlier web teleop commands because it
                # uses the same ROS publisher.
                self.teleop_pub.publish(Twist())
                with self.state_lock:
                    formation = int(self.leader_cmd['formation']) \
                        if self.leader_cmd is not None else 0
                    if formation == LeaderCmd.FORMATION_CIRCLE_SHOW:
                        formation = LeaderCmd.FORMATION_TRIANGLE
                    self.last_teleop = {
                        'vx': 0.0,
                        'wz': 0.0,
                        'stamp': rospy.Time.now().to_sec()
                    }
                mode_result = self._call_set_mode(
                    2, formation, 0.0, 0.0, 0.0)
                if not mode_result.get('ok', False):
                    return mode_result

                goal = PoseStamped()
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = self.frame
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = 0.0
                goal.pose.orientation.z = math.sin(yaw * 0.5)
                goal.pose.orientation.w = math.cos(yaw * 0.5)
                self.nav_goal_pub.publish(goal)
                with self.state_lock:
                    self.last_nav_goal = _pose_to_dict(
                        x, y, yaw, 'web_goal', goal.header.stamp.to_sec(), True)
                return {'ok': True, 'message': 'nav goal published'}
            if action == 'clear_trails':
                with self.state_lock:
                    for trail in self.trails.values():
                        del trail[:]
                return {'ok': True, 'message': 'trails cleared'}
            if action == 'toggle_avoidance':
                with self.state_lock:
                    self.avoidance_enabled = not self.avoidance_enabled
                    value = self.avoidance_enabled
                self.avoidance_pub.publish(Bool(data=value))
                return {'ok': True, 'message': 'avoidance set to %s' % value}
            if action == 'set_control_mode':
                value = str(payload.get('value', 'body_orbit'))
                if value not in ('body_orbit', 'wheeltec_global'):
                    return {'ok': False, 'message': 'unsupported control mode'}
                with self.state_lock:
                    self.control_mode = value
                self.control_mode_pub.publish(String(data=value))
                return {'ok': True, 'message': 'control mode set to %s' % value}
            return {'ok': False, 'message': 'unsupported action'}
        except Exception as exc:
            return {'ok': False, 'message': str(exc)}

    def _call_set_mode(self, mode, formation, offset_x, offset_y, offset_yaw):
        try:
            rospy.wait_for_service(self.set_mode_srv_name, timeout=1.0)
            proxy = rospy.ServiceProxy(self.set_mode_srv_name, SetMode)
            resp = proxy(mode=mode, formation=formation,
                         offset_x=offset_x, offset_y=offset_y, offset_yaw=offset_yaw)
            return {'ok': bool(resp.success), 'message': resp.message}
        except Exception as exc:
            return {'ok': False, 'message': str(exc)}

    def _call_set_formation(self, formation):
        try:
            rospy.wait_for_service(self.set_formation_srv_name, timeout=1.0)
            proxy = rospy.ServiceProxy(self.set_formation_srv_name, SetFormation)
            resp = proxy(formation=formation)
            return {'ok': bool(resp.success), 'message': resp.message}
        except Exception as exc:
            return {'ok': False, 'message': str(exc)}

    def shutdown(self):
        try:
            self.http_server.shutdown()
            self.http_server.server_close()
        except Exception:
            pass


def main():
    rospy.init_node('cluster_web_visualizer', anonymous=False)
    ClusterWebVisualizer()
    rospy.spin()


if __name__ == '__main__':
    main()
