/*
 * indexJS.h
 * HTML and Javascript code for mecanum drive web interface.
 * 
 * @author Ethan J. Musser
 */

#ifndef INDEXJS_H
#define INDEXJS_H

#include <string>

const char body[] PROGMEM = R"===(
<!DOCTYPE html>
<html>
<head>
    <style>
        .header {
            width: 100%;
            text-align: center;
            font-family: sans-serif;
        }
        .body {
            text-align: center;
            font-family: sans-serif;
        }
        .row {
            display: table;
            width: 100%;
            table-layout: fixed;
            border-spacing: 10px;
        }
        .section {
            display: table-cell;
            border-spacing: 10px;
            outline: 0.1em solid black;
            border-radius: 4px;
        }
        .block {
            display: table-cell;
            background-color: rgba(0, 0, 0, .1);
            border-spacing: 10px;
            border-radius: 4px;
        }
        .motor_block {
            display: table-cell;
            background-color: #f0ad4e;
            border-spacing: 10px;
            border-radius: 4px;
        }
        .status_table {
            width: 100%;
        }
        .parameter_col {
            width: 50%;
        }
        .value_col {
            width: 50%;
        }
        td:nth-child(1) {
            font-weight: bold;
            text-align: right;
        }
        .btn_div {
            display: table-cell;
        }
        .button {
            background-color: #4CAF50; /* Green */
            border: none;
            color: white;
            padding: 5px 5px;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            opacity: 0.8;
            border-radius: 4px;
            font-size: 16px;
            width: 95%;
            height: 60px;
        }
        .button:hover {
            opacity: 1;
        }
        .info {
            background-color: #5bc0de;
        }
        .plain {
            background-color: #aaaaaa;
        }
        .warn {
            background-color: #f0ad4e;
        }
        .danger {
            background-color: #bb2124;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>Lab 4.2 - Mobile Base</h1>
        <h2>Group 28: Ethan Donlon, Miriam Glickman, & Ethan Musser</h2>
    </div>
    <div class="body">
        <div class="row">
            <div class="section" title="status" style="width: 33%;">
                <div class="row" title="base">
                    <div class="block" title="position" style="width: 50%;">
                        <h3>Base</h3>
                        <div class="row">
                            <div class="motor_block">
                                <h4>Linear</h4>
                                <span id="base_linearspeed_label"></span>
                                <span id="base_lineardirection_label"></span>
                            </div>
                            <div class="motor_block">
                                <h4>Angular</h4>
                                <span id="base_angularspeed_label"></span>
                                <span id="base_angulardirection_label"></span>
                            </div>
                        </div>
                    </div>
                </div>
                <div class="row" title="motors">
                    <div class="block" title="motors" style="width: 50%;">
                        <h3>Motors</h3>
                        <div class="row">
                            <div class="motor_block" style="width: 50%;">
                                <h4>Motor 1</h4>
                                <span id="motor1_speed_label"></span>
                                <span id="motor1_direction_label"></span>
                            </div>
                            <div class="motor_block" style="width: 50%;">
                                <h4>Motor 2</h4>
                                <span id="motor2_speed_label"></span>
                                <span id="motor2_direction_label"></span>
                            </div>
                        </div>
                        <div class="row">
                            <div class="motor_block" style="width: 50%;">
                                <h4>Motor 3</h4>
                                <span id="motor3_speed_label"></span>
                                <span id="motor3_direction_label"></span>
                            </div>
                            <div class="motor_block" style="width: 50%;">
                                <h4>Motor 4</h4>
                                <span id="motor4_speed_label"></span>
                                <span id="motor4_direction_label"></span>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            <div class="section" title="control" style="width: auto;">
                <div class="row" title="movement_and_speed">
                    <div class="block" title="movement_ctrl" style="width: 60%;">
                        <h3>Movement</h3>
                        <div class="btn_panel">
                            <div class="row">
                                <div class="btn_div" style="width: 33%;">
                                    <button class="button info" id="rotate_left_btn" onmousedown="rot_left_btn_hit()" onmouseup="mvmt_btn_release()" onmousedown="rot_left_btn_hit()" onmouseup="mvmt_btn_release()">Rotate Left<br><kbd>Q</kbd></button>
                                </div>
                                <div class="btn_div" style="width: 33%;">
                                    <button class="button" id="move_forward_btn" onmousedown="move_fwd_btn_hit()" onmouseup="mvmt_btn_release()">Forward<br><kbd>W</kbd></button>
                                </div>
                                <div class="btn_div" style="width: 33%;">
                                    <button class="button info" id="rotate_right_btn" onmousedown="rot_right_btn_hit()" onmouseup="mvmt_btn_release()">Rotate Right<br><kbd>E</kbd></button>
                                </div>
                            </div>
                            <div class="row">
                                <div class="btn_div" style="width: 33%;">
                                    <button class="button" id="move_left_btn" onmousedown="move_left_btn_hit()" onmouseup="mvmt_btn_release()">Left<br><kbd>A</kbd></button>
                                </div>
                                <div class="btn_div" style="width: 33%;">
                                    <button class="button" id="move_backward_btn" onmousedown="move_bkwd_btn_hit()" onmouseup="mvmt_btn_release()">Backward<br><kbd>S</kbd></button>
                                </div>
                                <div class="btn_div" style="width: 33%;">
                                    <button class="button" id="move_right_btn" onmousedown="move_right_btn_hit()" onmouseup="mvmt_btn_release()">Right<br><kbd>D</kbd></button>
                                </div>
                            </div>
                        </div>
                    </div>
                    <div class="block" title="speed_ctrl" style="width: 40%;">
                        <h3>Speed</h3>
                        <div class="btn_panel">
                            <div class="row">
                                <div class="btn_div" style="width: 50%;">
                                    <button class="button" id="faster_speed_btn" onmousedown="spd_up_btn_hit()">Faster<br><kbd>&uparrow;</kbd></button>
                                </div>
                                <div class="btn_div" style="width: 50%;">
                                    <button class="button warn" id="full_speed_btn" onmousedown="spd_full_btn_hit()">Full<br><kbd>PgUp</kbd></button>
                                </div>
                            </div>
                            <div class="row">
                                <div class="btn_div" style="width: 50%;">
                                    <button class="button" id="slower_speed_btn" onmousedown="spd_dwn_btn_hit()">Slower<br><kbd>&downarrow;</kbd></button>
                                </div>
                                <div class="btn_div" style="width: 50%;">
                                    <button class="button warn" id="zero_speed_btn" onmousedown="spd_zero_btn_hit()">Zero<br><kbd>PgDn</kbd></button>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>
    <script>
        /**
         * Messages
         */
        var esp32_msg = [];
        var esp32_status = [];

        /**
         * Listeners
         */
        document.addEventListener('keydown', keyDownHandler);
        document.addEventListener('keyup', keyUpHandler);

        /**
         * Movement Button Hits
         */
        function move_fwd_btn_hit() {
            var xhttp = new XMLHttpRequest();
            xhttp.open("GET", "move_fwd_btn_hit", true);
            xhttp.send();
        }  
        function move_bkwd_btn_hit() {
            var xhttp = new XMLHttpRequest();
            xhttp.open("GET", "move_bkwd_btn_hit", true);
            xhttp.send();
        }  
        function move_left_btn_hit() {
            var xhttp = new XMLHttpRequest();
            xhttp.open("GET", "move_left_btn_hit", true);
            xhttp.send();
        }  
        function move_right_btn_hit() {
            var xhttp = new XMLHttpRequest();
            xhttp.open("GET", "move_right_btn_hit", true);
            xhttp.send();
        }  
        function rot_left_btn_hit() {
            var xhttp = new XMLHttpRequest();
            xhttp.open("GET", "rot_left_btn_hit", true);
            xhttp.send();
        }  
        function rot_right_btn_hit() {
            var xhttp = new XMLHttpRequest();
            xhttp.open("GET", "rot_right_btn_hit", true);
            xhttp.send();
        }  
        // Movement Button Release
        function mvmt_btn_release() {
            var xhttp = new XMLHttpRequest();
            xhttp.open("GET", "mvmt_btn_release", true);
            xhttp.send();
        }  
        // Speed Button Hits
        function spd_up_btn_hit() {
            var xhttp = new XMLHttpRequest();
            xhttp.open("GET", "spd_up_btn_hit", true);
            xhttp.send();
        }  
        function spd_dwn_btn_hit() {
            var xhttp = new XMLHttpRequest();
            xhttp.open("GET", "spd_dwn_btn_hit", true);
            xhttp.send();
        }  
        function spd_full_btn_hit() {
            var xhttp = new XMLHttpRequest();
            xhttp.open("GET", "spd_full_btn_hit", true);
            xhttp.send();
        }  
        function spd_zero_btn_hit() {
            var xhttp = new XMLHttpRequest();
            xhttp.open("GET", "spd_zero_btn_hit", true);
            xhttp.send();
        }  

        /**
         * Key Events
         */      
        var getKeyboardCode = function(event) {
            var code;
            if (event.repeat) return 0;
            if (event.key !== undefined) code = event.key;
            else if (event.keyCode !== undefined) code = event.keyCode;
            else if (event.which !== undefined) code = event.which;
            return code;
        };

        function keyDownHandler(event){
            if (event.defaultPrevented) return;
            var code = getKeyboardCode(event);
            if (code == 81 || code == 'q' || code == 'KeyQ'){
                rot_left_btn_hit();
            } else if (code == 87 || code == 'w' || code == 'KeyW') {
                move_fwd_btn_hit();
            } else if (code == 69 || code == 'e' || code == 'KeyE') {
                rot_right_btn_hit();
            } else if (code == 65 || code == 'a' || code == 'KeyA') {
                move_left_btn_hit();
            } else if (code == 83 || code == 's' || code == 'KeyS') {
                move_bkwd_btn_hit();
            } else if (code == 68 || code == 'd' || code == 'KeyD') {
                move_right_btn_hit();
            } else if (code == 38 || code == 'ArrowUp') {
                spd_up_btn_hit();
                event.preventDefault();
            } else if (code == 40 || code == 'ArrowDown') {
                spd_dwn_btn_hit();
                event.preventDefault();
            } else if (code == 80 || code == 'p' || code == 'KeyP') {
                spd_full_btn_hit();
                event.preventDefault();
            } else if (code == 76 || code == 'l' || code == 'KeyL') {
                spd_zero_btn_hit();
                event.preventDefault();
            }
        }

        function keyUpHandler(event){
            if (event.defaultPrevented) return;
            var code = getKeyboardCode(event);
            if (code == 81 || code == 87 || code == 69 || code == 65 || code == 83 || code == 68 ||
                code == 'q' || code == 'w' || code == 'e' || code == 'a' || code == 's' || code == 'd' ||
                code == 'KeyQ' || code == 'KeyW' || code == 'KeyE' || code == 'KewA' || code == 'KeyS' || code == 'KeyD') {
                mvmt_btn_release();
            }
            event.preventDefault();
        }
       
    </script>
</body>
</html>
)===";

#endif  // INDEXJS_H
