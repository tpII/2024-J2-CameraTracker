def convert_to_quoted_lines(input_string):
    # Split the input into lines
    lines = input_string.splitlines()
    # Add quotes to each line and escape double quotes
    quoted_lines = ['"{}"'.format(line.replace('"', '\\"')) for line in lines]
    # Join the quoted lines with newline and continuation character
    return " \r\n".join(quoted_lines)

def save_to_file(output, filename):
    with open(filename, "w") as file:
        file.write(output)

# Example input
html_code = """<!doctype html>
<html>
    <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width,initial-scale=1">
        <title>Camera Tracker</title>
        <style>
            body {
                font-family: Arial,Helvetica,sans-serif;
                background: #181818;
                color: #EFEFEF;
                font-size: 16px
            }

            h2 {
                font-size: 18px
            }

            section.main {
                display: flex
            }

            #menu,section.main {
                flex-direction: column
            }

            #menu {
                display: none;
                flex-wrap: nowrap;
                min-width: 340px;
                background: #363636;
                padding: 8px;
                border-radius: 4px;
                margin-top: -10px;
                margin-right: 10px;
            }

            #content {
                display: flex;
                flex-wrap: wrap;
                align-items: stretch
            }

            figure {
                padding: 0px;
                margin: 0;
                -webkit-margin-before: 0;
                margin-block-start: 0;
                -webkit-margin-after: 0;
                margin-block-end: 0;
                -webkit-margin-start: 0;
                margin-inline-start: 0;
                -webkit-margin-end: 0;
                margin-inline-end: 0
            }

            figure img {
                display: block;
                width: 100%;
                height: auto;
                border-radius: 4px;
                margin-top: 8px;
            }

            @media (min-width: 800px) and (orientation:landscape) {
                #content {
                    display:flex;
                    flex-wrap: nowrap;
                    align-items: stretch
                }

                figure img {
                    display: block;
                    max-width: 100%;
                    max-height: calc(100vh - 40px);
                    width: auto;
                    height: auto
                }

                figure {
                    padding: 0 0 0 0px;
                    margin: 0;
                    -webkit-margin-before: 0;
                    margin-block-start: 0;
                    -webkit-margin-after: 0;
                    margin-block-end: 0;
                    -webkit-margin-start: 0;
                    margin-inline-start: 0;
                    -webkit-margin-end: 0;
                    margin-inline-end: 0
                }
            }

            section#buttons {
                display: flex;
                flex-wrap: nowrap;
                justify-content: space-between
            }

            #nav-toggle {
                cursor: pointer;
                display: block
            }

            #nav-toggle-cb {
                outline: 0;
                opacity: 0;
                width: 0;
                height: 0
            }

            #nav-toggle-cb:checked+#menu {
                display: flex
            }

            .input-group {
                display: flex;
                flex-wrap: nowrap;
                line-height: 22px;
                margin: 5px 0
            }

            .input-group>label {
                display: inline-block;
                padding-right: 10px;
                min-width: 47%
            }

            .input-group input,.input-group select,.input-group span {
                flex-grow: 1
            }

            .range-max,.range-min {
                display: inline-block;
                padding: 0 5px
            }

            button, .button {
                display: block;
                margin: 5px;
                padding: 0 12px;
                border: 0;
                line-height: 28px;
                cursor: pointer;
                color: #fff;
                background: #ff3034;
                border-radius: 5px;
                font-size: 16px;
                outline: 0
            }

            button:hover {
                background: #ff494d
            }

            button:active {
                background: #f21c21
            }

            button.disabled {
                cursor: default;
                background: #a0a0a0
            }

            input[type=range] {
                -webkit-appearance: none;
                width: 100%;
                height: 22px;
                background: #363636;
                cursor: pointer;
                margin: 0
            }

            input[type=range]:focus {
                outline: 0
            }

            input[type=range]::-webkit-slider-runnable-track {
                width: 100%;
                height: 2px;
                cursor: pointer;
                background: #EFEFEF;
                border-radius: 0;
                border: 0 solid #EFEFEF
            }

            input[type=range]::-webkit-slider-thumb {
                border: 1px solid rgba(0,0,30,0);
                height: 22px;
                width: 22px;
                border-radius: 50px;
                background: #ff3034;
                cursor: pointer;
                -webkit-appearance: none;
                margin-top: -11.5px
            }

            input[type=range]:focus::-webkit-slider-runnable-track {
                background: #EFEFEF
            }

            input[type=range]::-moz-range-track {
                width: 100%;
                height: 2px;
                cursor: pointer;
                background: #EFEFEF;
                border-radius: 0;
                border: 0 solid #EFEFEF
            }

            input[type=range]::-moz-range-thumb {
                border: 1px solid rgba(0,0,30,0);
                height: 22px;
                width: 22px;
                border-radius: 50px;
                background: #ff3034;
                cursor: pointer
            }

            input[type=range]::-ms-track {
                width: 100%;
                height: 2px;
                cursor: pointer;
                background: 0 0;
                border-color: transparent;
                color: transparent
            }

            input[type=range]::-ms-fill-lower {
                background: #EFEFEF;
                border: 0 solid #EFEFEF;
                border-radius: 0
            }

            input[type=range]::-ms-fill-upper {
                background: #EFEFEF;
                border: 0 solid #EFEFEF;
                border-radius: 0
            }

            input[type=range]::-ms-thumb {
                border: 1px solid rgba(0,0,30,0);
                height: 22px;
                width: 22px;
                border-radius: 50px;
                background: #ff3034;
                cursor: pointer;
                height: 2px
            }

            input[type=range]:focus::-ms-fill-lower {
                background: #EFEFEF
            }

            input[type=range]:focus::-ms-fill-upper {
                background: #363636
            }

            .switch {
                display: block;
                position: relative;
                line-height: 22px;
                font-size: 16px;
                height: 22px
            }

            .switch input {
                outline: 0;
                opacity: 0;
                width: 0;
                height: 0
            }

            .slider {
                width: 50px;
                height: 22px;
                border-radius: 22px;
                cursor: pointer;
                background-color: grey
            }

            .slider,.slider:before {
                display: inline-block;
                transition: .4s
            }

            .slider:before {
                position: relative;
                content: "";
                border-radius: 50%;
                height: 16px;
                width: 16px;
                left: 4px;
                top: 3px;
                background-color: #fff
            }

            input:checked+.slider {
                background-color: #ff3034
            }

            input:checked+.slider:before {
                -webkit-transform: translateX(26px);
                transform: translateX(26px)
            }

            select {
                border: 1px solid #363636;
                font-size: 14px;
                height: 22px;
                outline: 0;
                border-radius: 5px
            }

            .image-container {
                position: relative;
                min-width: 160px
            }

            .close {
                position: absolute;
                right: 5px;
                top: 5px;
                background: #ff3034;
                width: 16px;
                height: 16px;
                border-radius: 100px;
                color: #fff;
                text-align: center;
                line-height: 18px;
                cursor: pointer
            }

            .hidden {
                display: none
            }

            .save {
                position: absolute;
                right: 25px;
                top: 0px;
                height: 16px;
                line-height: 16px;
                padding: 0 4px;
                text-decoration: none;
                cursor: pointer
            }

            input[type=text] {
                border: 1px solid #363636;
                font-size: 14px;
                height: 20px;
                margin: 1px;
                outline: 0;
                border-radius: 5px
            }

            .inline-button {
                line-height: 20px;
                margin: 2px;
                padding: 1px 4px 2px 4px;
            }

            label.toggle-section-label {
                cursor: pointer;
                display: block
            }

            input.toggle-section-button {
                outline: 0;
                opacity: 0;
                width: 0;
                height: 0
            }

            input.toggle-section-button:checked+section.toggle-section {
                display: none
            }

        </style>
    </head>
    <body>
        <section class="main">
            <div id="logo">
                <label for="nav-toggle-cb" id="nav-toggle">&#9776;&nbsp;&nbsp;Toggle settings</label>
            </div>
            <div id="content">
                <div id="sidebar">
                    <input type="checkbox" id="nav-toggle-cb" checked="checked">
                    <nav id="menu">

                        <section id="xclk-section" class="nothidden">
                            <div class="input-group" id="set-xclk-group">
                                <label for="set-xclk">XCLK MHz</label>
                                <div class="text">
                                    <input id="xclk" type="text" minlength="1" maxlength="2" size="2" value="20">
                                </div>
                                <button class="inline-button" id="set-xclk">Set</button>
                            </div>
                        </section>

                        <div class="input-group" id="framesize-group">
                            <label for="framesize">Resolution</label>
                            <select id="framesize" class="default-action">
                                <!-- 5MP -->
                                <option value="21" selected="selected">QSXGA(2560x1920)</option>
                                <option value="20">P FHD(1080x1920)</option>
                                <option value="19">WQXGA(2560x1600)</option>
                                <option value="18">QHD(2560x1440)</option>
                                <!-- 3MP -->
                                <option value="17">QXGA(2048x1564)</option>
                                <option value="16">P 3MP(864x1564)</option>
                                <option value="15">P HD(720x1280)</option>
                                <option value="14">FHD(1920x1080)</option>
                                <!-- 2MP -->
                                <option value="13">UXGA(1600x1200)</option>
                                <option value="12">SXGA(1280x1024)</option>
                                <option value="11">HD(1280x720)</option>
                                <option value="10">XGA(1024x768)</option>
                                <option value="9">SVGA(800x600)</option>
                                <option value="8">VGA(640x480)</option>
                                <option value="7">HVGA(480x320)</option>
                                <option value="6">CIF(400x296)</option>
                                <option value="5">QVGA(320x240)</option>
                                <option value="4">240x240</option>
                                <option value="3">HQVGA(240x176)</option>
                                <option value="2">QCIF(176x144)</option>
                                <option value="1">QQVGA(160x120)</option>
                                <option value="0">96x96</option>
                            </select>
                        </div>
                        <div class="input-group" id="quality-group">
                            <label for="quality">Quality</label>
                            <div class="range-min">4</div>
                            <input type="range" id="quality" min="4" max="63" value="10" class="default-action">
                            <div class="range-max">63</div>
                        </div>
                        <div class="input-group" id="brightness-group">
                            <label for="brightness">Brightness</label>
                            <div class="range-min">-3</div>
                            <input type="range" id="brightness" min="-3" max="3" value="0" class="default-action">
                            <div class="range-max">3</div>
                        </div>
                        <div class="input-group" id="contrast-group">
                            <label for="contrast">Contrast</label>
                            <div class="range-min">-3</div>
                            <input type="range" id="contrast" min="-3" max="3" value="0" class="default-action">
                            <div class="range-max">3</div>
                        </div>
                        <div class="input-group" id="saturation-group">
                            <label for="saturation">Saturation</label>
                            <div class="range-min">-4</div>
                            <input type="range" id="saturation" min="-4" max="4" value="0" class="default-action">
                            <div class="range-max">4</div>
                        </div>
                        <div class="input-group" id="sharpness-group">
                            <label for="sharpness">Sharpness</label>
                            <div class="range-min">-3</div>
                            <input type="range" id="sharpness" min="-3" max="3" value="0" class="default-action">
                            <div class="range-max">3</div>
                        </div>
                        <div class="input-group" id="denoise-group">
                            <label for="denoise">De-Noise</label>
                            <div class="range-min">Auto</div>
                            <input type="range" id="denoise" min="0" max="8" value="0" class="default-action">
                            <div class="range-max">8</div>
                        </div>
                        <div class="input-group" id="ae_level-group">
                            <label for="ae_level">Exposure Level</label>
                            <div class="range-min">-5</div>
                            <input type="range" id="ae_level" min="-5" max="5" value="0" class="default-action">
                            <div class="range-max">5</div>
                        </div>
                        <div class="input-group" id="gainceiling-group">
                            <label for="gainceiling">Gainceiling</label>
                            <div class="range-min">0</div>
                            <input type="range" id="gainceiling" min="0" max="511" value="0" class="default-action">
                            <div class="range-max">511</div>
                        </div>
                        <div class="input-group" id="special_effect-group">
                            <label for="special_effect">Special Effect</label>
                            <select id="special_effect" class="default-action">
                                <option value="0" selected="selected">No Effect</option>
                                <option value="1">Negative</option>
                                <option value="2">Grayscale</option>
                                <option value="3">Red Tint</option>
                                <option value="4">Green Tint</option>
                                <option value="5">Blue Tint</option>
                                <option value="6">Sepia</option>
                            </select>
                        </div>
                        <div class="input-group" id="awb-group">
                            <label for="awb">AWB Enable</label>
                            <div class="switch">
                                <input id="awb" type="checkbox" class="default-action" checked="checked">
                                <label class="slider" for="awb"></label>
                            </div>
                        </div>
                        <div class="input-group" id="dcw-group">
                            <label for="dcw">Advanced AWB</label>
                            <div class="switch">
                                <input id="dcw" type="checkbox" class="default-action" checked="checked">
                                <label class="slider" for="dcw"></label>
                            </div>
                        </div>
                        <div class="input-group" id="awb_gain-group">
                            <label for="awb_gain">Manual AWB</label>
                            <div class="switch">
                                <input id="awb_gain" type="checkbox" class="default-action" checked="checked">
                                <label class="slider" for="awb_gain"></label>
                            </div>
                        </div>
                        <div class="input-group" id="wb_mode-group">
                            <label for="wb_mode">AWB Mode</label>
                            <select id="wb_mode" class="default-action">
                                <option value="0" selected="selected">Auto</option>
                                <option value="1">Sunny</option>
                                <option value="2">Cloudy</option>
                                <option value="3">Office</option>
                                <option value="4">Home</option>
                            </select>
                        </div>
                        <div class="input-group" id="aec-group">
                            <label for="aec">AEC Enable</label>
                            <div class="switch">
                                <input id="aec" type="checkbox" class="default-action" checked="checked">
                                <label class="slider" for="aec"></label>
                            </div>
                        </div>
                        <div class="input-group" id="aec_value-group">
                            <label for="aec_value">Manual Exposure</label>
                            <div class="range-min">0</div>
                            <input type="range" id="aec_value" min="0" max="1920" value="320" class="default-action">
                            <div class="range-max">1920</div>
                        </div>
                        <div class="input-group" id="aec2-group">
                            <label for="aec2">Night Mode</label>
                            <div class="switch">
                                <input id="aec2" type="checkbox" class="default-action" checked="checked">
                                <label class="slider" for="aec2"></label>
                            </div>
                        </div>
                        <div class="input-group" id="agc-group">
                            <label for="agc">AGC</label>
                            <div class="switch">
                                <input id="agc" type="checkbox" class="default-action" checked="checked">
                                <label class="slider" for="agc"></label>
                            </div>
                        </div>
                        <div class="input-group hidden" id="agc_gain-group">
                            <label for="agc_gain">Gain</label>
                            <div class="range-min">1x</div>
                            <input type="range" id="agc_gain" min="0" max="64" value="5" class="default-action">
                            <div class="range-max">64x</div>
                        </div>
                        <div class="input-group" id="raw_gma-group">
                            <label for="raw_gma">GMA Enable</label>
                            <div class="switch">
                                <input id="raw_gma" type="checkbox" class="default-action" checked="checked">
                                <label class="slider" for="raw_gma"></label>
                            </div>
                        </div>
                        <div class="input-group" id="lenc-group">
                            <label for="lenc">Lens Correction</label>
                            <div class="switch">
                                <input id="lenc" type="checkbox" class="default-action" checked="checked">
                                <label class="slider" for="lenc"></label>
                            </div>
                        </div>
                        <div class="input-group" id="hmirror-group">
                            <label for="hmirror">H-Mirror</label>
                            <div class="switch">
                                <input id="hmirror" type="checkbox" class="default-action" checked="checked">
                                <label class="slider" for="hmirror"></label>
                            </div>
                        </div>
                        <div class="input-group" id="vflip-group">
                            <label for="vflip">V-Flip</label>
                            <div class="switch">
                                <input id="vflip" type="checkbox" class="default-action" checked="checked">
                                <label class="slider" for="vflip"></label>
                            </div>
                        </div>
                        <div class="input-group" id="bpc-group">
                            <label for="bpc">BPC</label>
                            <div class="switch">
                                <input id="bpc" type="checkbox" class="default-action">
                                <label class="slider" for="bpc"></label>
                            </div>
                        </div>
                        <div class="input-group" id="wpc-group">
                            <label for="wpc">WPC</label>
                            <div class="switch">
                                <input id="wpc" type="checkbox" class="default-action" checked="checked">
                                <label class="slider" for="wpc"></label>
                            </div>
                        </div>
                        <div class="input-group" id="colorbar-group">
                            <label for="colorbar">Color Bar</label>
                            <div class="switch">
                                <input id="colorbar" type="checkbox" class="default-action">
                                <label class="slider" for="colorbar"></label>
                            </div>
                        </div>
                        <div class="input-group" id="face_detect-group">
                            <label for="face_detect">Face Detection</label>
                            <div class="switch">
                                <input id="face_detect" type="checkbox" class="default-action">
                                <label class="slider" for="face_detect"></label>
                            </div>
                        </div>
                        <div class="input-group" id="face_recognize-group">
                            <label for="face_recognize">Face Recognition</label>
                            <div class="switch">
                                <input id="face_recognize" type="checkbox" class="default-action">
                                <label class="slider" for="face_recognize"></label>
                            </div>
                        </div>


                        <section id="buttons">
                            <button id="get-still">Get Still</button>
                            <button id="toggle-stream">Start Stream</button>
                            <button id="face_enroll" class="disabled" disabled="disabled">Enroll Face</button>
                        </section>

                        <div style="margin-top: 8px;"><center><span style="font-weight: bold;">Advanced Settings</span></center></div>
                        <hr style="width:100%">
                        <label for="nav-toggle-reg" class="toggle-section-label">&#9776;&nbsp;&nbsp;Register Get/Set</label><input type="checkbox" id="nav-toggle-reg" class="hidden toggle-section-button" checked="checked">
                        <section class="toggle-section">
                            <!--h4>Set Register</h4-->
                            <div class="input-group" id="set-reg-group">
                                <label for="set-reg">Reg, Mask, Value</label>
                                <div class="text">
                                    <input id="reg-addr" type="text" minlength="4" maxlength="6" size="6" value="0x3008">
                                </div>
                                <div class="text">
                                    <input id="reg-mask" type="text" minlength="4" maxlength="4" size="4" value="0xff">
                                </div>
                                <div class="text">
                                    <input id="reg-value" type="text" minlength="4" maxlength="4" size="4" value="0x02">
                                </div>
                                <button class="inline-button" id="set-reg">Set</button>
                            </div>
                            <hr style="width:50%">
                            <!--h4>Get Register</h4-->
                            <div class="input-group" id="get-reg-group">
                                <label for="get-reg">Reg, Mask</label>
                                <div class="text">
                                    <input id="get-reg-addr" type="text" minlength="4" maxlength="6" size="6" value="0x3008">
                                </div>
                                <div class="text">
                                    <input id="get-reg-mask" type="text" minlength="4" maxlength="6" size="6" value="0x00ff">
                                </div>
                                <button class="inline-button" id="get-reg">Get</button>
                            </div>
                            <div class="input-group">
                                <label for="get-reg-value">Value</label>
                                <div class="text">
                                    <span id="get-reg-value">0x1234</span>
                                </div>
                            </div>
                        </section>
                        <hr style="width:100%">
                        <label for="nav-toggle-pll" class="toggle-section-label">&#9776;&nbsp;&nbsp;PLL</label><input type="checkbox" id="nav-toggle-pll" class="hidden toggle-section-button" checked="checked">
                        <section class="toggle-section">
                            <div class="input-group" id="bypass-pll-group">
                                <label for="bypass-pll">Bypass PLL</label>
                                <div class="switch">
                                    <input id="bypass-pll" type="checkbox">
                                    <label class="slider" for="bypass-pll"></label>
                                </div>
                            </div>
                            <div class="input-group" id="set-pre-pll-group">
                                <label for="pre-pll">Pre Div</label>
                                <select id="pre-pll">
                                    <option value="1" selected="selected">1x</option>
                                    <option value="2">2x</option>
                                    <option value="3">3x</option>
                                    <option value="4">4x</option>
                                    <option value="5">1.5x</option>
                                    <option value="6">6x</option>
                                    <option value="7">2.5x</option>
                                    <option value="8">8x</option>
                                </select>
                            </div>
                            <div class="input-group" id="set-mul-pll-group">
                                <label for="mul-pll">Multiplier (4 - 252)</label>
                                <div class="text">
                                    <input id="mul-pll" type="text" minlength="1" maxlength="3" size="18" value="10">
                                </div>
                            </div>
                            <div class="input-group" id="root-pll-group">
                                <label for="root-pll">Root Div</label>
                                <select id="root-pll">
                                    <option value="0" selected="selected">1x</option>
                                    <option value="1">2x</option>
                                </select>
                            </div>
                            <div class="input-group" id="set-sys-pll-group">
                                <label for="sys-pll">System Div (0 - 15)</label>
                                <div class="text">
                                    <input id="sys-pll" type="text" minlength="1" maxlength="2" size="18" value="1">
                                </div>
                            </div>
                            <div class="input-group" id="set-seld5-pll-group">
                                <label for="seld5-pll">PCLK Root Div</label>
                                <select id="seld5-pll">
                                    <option value="0">1x</option>
                                    <option value="1" selected="selected">2x</option>
                                    <option value="2">4x</option>
                                    <option value="3">8x</option>
                                </select>
                            </div>
                            <div class="input-group" id="pclk-en-group">
                                <label for="pclk-en">PCLK Manual</label>
                                <div class="switch">
                                    <input id="pclk-en" type="checkbox" checked="checked">
                                    <label class="slider" for="pclk-en"></label>
                                </div>
                            </div>
                            <div class="input-group" id="set-pclk-pll-group">
                                <label for="pclk-pll">PCLK Div (0 - 31)</label>
                                <select id="pclk-pll">
                                    <option value="1">1x</option>
                                    <option value="2">2x</option>
                                    <option value="4" selected="selected">4x</option>
                                    <option value="8">8x</option>
                                    <option value="16">16x</option>
                                </select>
                            </div>
                            <button id="set-pll">Set PLL</button>
                        </section>
                        <hr style="width:100%">
                        <label for="nav-toggle-win" class="toggle-section-label">&#9776;&nbsp;&nbsp;Window</label><input type="checkbox" id="nav-toggle-win" class="hidden toggle-section-button" checked="checked">
                        <section class="toggle-section">
                            <div class="input-group" id="set-start-res-group">
                                <label for="start-x">Adress Start</label>
                                <div class="text">
                                    X:<input id="start-x" type="text" minlength="1" maxlength="4" size="6" value="0">
                                </div>
                                <div class="text">
                                    Y:<input id="start-y" type="text" minlength="1" maxlength="4" size="6" value="0">
                                </div>
                            </div>
                            <div class="input-group" id="set-end-res-group">
                                <label for="end-x">Adress End</label>
                                <div class="text">
                                    X:<input id="end-x" type="text" minlength="1" maxlength="4" size="6" value="2623">
                                </div>
                                <div class="text">
                                    Y:<input id="end-y" type="text" minlength="1" maxlength="4" size="6" value="1951">
                                </div>
                            </div>
                            <div class="input-group" id="set-offset-res-group">
                                <label for="offset-x">Offset</label>
                                <div class="text">
                                    X:<input id="offset-x" type="text" minlength="1" maxlength="3" size="6" value="16">
                                </div>
                                <div class="text">
                                    Y:<input id="offset-y" type="text" minlength="1" maxlength="3" size="6" value="4">
                                </div>
                            </div>
                            <div class="input-group" id="set-total-res-group">
                                <label for="total-x">Total Size</label>
                                <div class="text">
                                    X:<input id="total-x" type="text" minlength="1" maxlength="4" size="6" value="2844">
                                </div>
                                <div class="text">
                                    Y:<input id="total-y" type="text" minlength="1" maxlength="4" size="6" value="1968">
                                </div>
                            </div>
                            <div class="input-group" id="set-output-res-group">
                                <label for="output-x">Output Size</label>
                                <div class="text">
                                    X:<input id="output-x" type="text" minlength="1" maxlength="4" size="6" value="2592">
                                </div>
                                <div class="text">
                                    Y:<input id="output-y" type="text" minlength="1" maxlength="4" size="6" value="1944">
                                </div>
                            </div>
                            <div class="input-group" id="scaling-group">
                                <label for="scaling">Scaling</label>
                                <div class="switch">
                                    <input id="scaling" type="checkbox">
                                    <label class="slider" for="scaling"></label>
                                </div>
                            </div>
                            <div class="input-group" id="binning-group">
                                <label for="binning">Binning</label>
                                <div class="switch">
                                    <input id="binning" type="checkbox">
                                    <label class="slider" for="binning"></label>
                                </div>
                            </div>
                            <button id="set-resolution">Set Resolution</button>
                        </section>
                        <hr style="width:100%">
                        <label for="nav-toggle-gamma" class="toggle-section-label">&#9776;&nbsp;&nbsp;Gamma</label><input type="checkbox" id="nav-toggle-gamma" class="hidden toggle-section-button" checked="checked">
                        <section class="toggle-section">
                            <div class="input-group"><label for="gamma-bias">Gamma Bias Plus</label><div class="switch"><input id="gamma-bias" type="checkbox" class="reg-action" reg="0x5480" offset="0" mask="0x01"><label class="slider" for="gamma-bias"></label></div></div>
                            <div class="input-group"><label for="gamma-1">Gamma 0</label><input type="range" id="gamma-1" min="0" max="255" value="0" class="reg-action" reg="0x5481" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="gamma-2">Gamma 1</label><input type="range" id="gamma-2" min="0" max="255" value="0" class="reg-action" reg="0x5482" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="gamma-3">Gamma 2</label><input type="range" id="gamma-3" min="0" max="255" value="0" class="reg-action" reg="0x5483" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="gamma-4">Gamma 3</label><input type="range" id="gamma-4" min="0" max="255" value="0" class="reg-action" reg="0x5484" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="gamma-5">Gamma 4</label><input type="range" id="gamma-5" min="0" max="255" value="0" class="reg-action" reg="0x5485" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="gamma-6">Gamma 5</label><input type="range" id="gamma-6" min="0" max="255" value="0" class="reg-action" reg="0x5486" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="gamma-7">Gamma 6</label><input type="range" id="gamma-7" min="0" max="255" value="0" class="reg-action" reg="0x5487" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="gamma-8">Gamma 7</label><input type="range" id="gamma-8" min="0" max="255" value="0" class="reg-action" reg="0x5488" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="gamma-9">Gamma 8</label><input type="range" id="gamma-9" min="0" max="255" value="0" class="reg-action" reg="0x5489" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="gamma-10">Gamma 9</label><input type="range" id="gamma-10" min="0" max="255" value="0" class="reg-action" reg="0x548a" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="gamma-11">Gamma 10</label><input type="range" id="gamma-11" min="0" max="255" value="0" class="reg-action" reg="0x548b" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="gamma-12">Gamma 11</label><input type="range" id="gamma-12" min="0" max="255" value="0" class="reg-action" reg="0x548c" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="gamma-13">Gamma 12</label><input type="range" id="gamma-13" min="0" max="255" value="0" class="reg-action" reg="0x548d" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="gamma-14">Gamma 13</label><input type="range" id="gamma-14" min="0" max="255" value="0" class="reg-action" reg="0x548e" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="gamma-15">Gamma 14</label><input type="range" id="gamma-15" min="0" max="255" value="0" class="reg-action" reg="0x548f" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="gamma-16">Gamma 15</label><input type="range" id="gamma-16" min="0" max="255" value="0" class="reg-action" reg="0x5490" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="get-gamma"></label><button class="inline-button" id="get-gamma">Get Gamma Config</button></div>
                        </section>
                        <hr style="width:100%">
                        <label for="nav-toggle-sde" class="toggle-section-label">&#9776;&nbsp;&nbsp;SDE</label><input type="checkbox" id="nav-toggle-sde" class="hidden toggle-section-button" checked="checked">
                        <section class="toggle-section">
                            <div class="input-group"><label for="sde2">Negative</label><div class="switch"><input id="sde2" type="checkbox" class="reg-action" reg="0x5580" offset="6" mask="0x01"><label class="slider" for="sde2"></label></div></div>
                            <div class="input-group"><label for="sde3">Gray</label><div class="switch"><input id="sde3" type="checkbox" class="reg-action" reg="0x5580" offset="5" mask="0x01"><label class="slider" for="sde3"></label></div></div>
                            <hr style="width:50%">
                            <div class="input-group"><label for="sde1">Fixed Y</label><div class="switch"><input id="sde1" type="checkbox" class="reg-action" reg="0x5580" offset="7" mask="0x01"><label class="slider" for="sde1"></label></div></div>
                            <div class="input-group"><label for="sde5">Fixed U</label><div class="switch"><input id="sde5" type="checkbox" class="reg-action" reg="0x5580" offset="3" mask="0x01"><label class="slider" for="sde5"></label></div></div>
                            <div class="input-group"><label for="sde4">Fixed V</label><div class="switch"><input id="sde4" type="checkbox" class="reg-action" reg="0x5580" offset="4" mask="0x01"><label class="slider" for="sde4"></label></div></div>
                            <div class="input-group"><label for="sde22">UV Thresh 1</label><input type="range" id="sde22" min="0" max="255" value="0" class="reg-action" reg="0x5589" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="sde23">UV Thresh 2</label><input type="range" id="sde23" min="0" max="511" value="0" class="reg-action" reg="0x558a" offset="0" mask="0x1ff"></div>
                            <hr style="width:50%">
                            <div class="input-group"><label for="sde8">Hue</label><div class="switch"><input id="sde8" type="checkbox" class="reg-action" reg="0x5580" offset="0" mask="0x01"><label class="slider" for="sde8"></label></div></div>
                            <div class="input-group"><label for="sde9">Hue Cos</label><input type="range" id="sde9" min="0" max="255" value="0" class="reg-action" reg="0x5581" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="sde17">Hue U Cos Neg</label><div class="switch"><input id="sde17" type="checkbox" class="reg-action" reg="0x5588" offset="4" mask="0x01"><label class="slider" for="sde17"></label></div></div>
                            <div class="input-group"><label for="sde16">Hue V Cos Neg</label><div class="switch"><input id="sde16" type="checkbox" class="reg-action" reg="0x5588" offset="5" mask="0x01"><label class="slider" for="sde16"></label></div></div>
                            <div class="input-group"><label for="sde10">Hue Sin</label><input type="range" id="sde10" min="0" max="255" value="0" class="reg-action" reg="0x5582" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="sde21">Hue U Sin Neg</label><div class="switch"><input id="sde21" type="checkbox" class="reg-action" reg="0x5588" offset="0" mask="0x01"><label class="slider" for="sde21"></label></div></div>
                            <div class="input-group"><label for="sde20">Hue V Sin Neg</label><div class="switch"><input id="sde20" type="checkbox" class="reg-action" reg="0x5588" offset="1" mask="0x01"><label class="slider" for="sde20"></label></div></div>
                            <hr style="width:50%">
                            <div class="input-group"><label for="sde6">Contrast</label><div class="switch"><input id="sde6" type="checkbox" class="reg-action" reg="0x5580" offset="2" mask="0x01"><label class="slider" for="sde6"></label></div></div>
                            <div class="input-group"><label for="sde13">Contrast Y Offset</label><input type="range" id="sde13" min="0" max="255" value="0" class="reg-action" reg="0x5585" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="sde19">Contrast Y Offset Neg</label><div class="switch"><input id="sde19" type="checkbox" class="reg-action" reg="0x5588" offset="2" mask="0x01"><label class="slider" for="sde19"></label></div></div>
                            <div class="input-group"><label for="sde15">Contrast Y Bright</label><input type="range" id="sde15" min="0" max="255" value="0" class="reg-action" reg="0x5587" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="sde18">Contrast Y Bright Neg</label><div class="switch"><input id="sde18" type="checkbox" class="reg-action" reg="0x5588" offset="3" mask="0x01"><label class="slider" for="sde18"></label></div></div>
                            <div class="input-group"><label for="sde14">Contrast Y Gain</label><input type="range" id="sde14" min="0" max="255" value="0" class="reg-action" reg="0x5586" offset="0" mask="0xff"></div>
                            <hr style="width:50%">
                            <div class="input-group"><label for="sde7">Saturation</label><div class="switch"><input id="sde7" type="checkbox" class="reg-action" reg="0x5580" offset="1" mask="0x01"><label class="slider" for="sde7"></label></div></div>
                            <div class="input-group"><label for="sde11">U Saturation</label><input type="range" id="sde11" min="0" max="255" value="0" class="reg-action" reg="0x5583" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="sde12">V Saturation</label><input type="range" id="sde12" min="0" max="255" value="0" class="reg-action" reg="0x5584" offset="0" mask="0xff"></div>
                        </section>
                        <hr style="width:100%">
                        <label for="nav-toggle-cmx" class="toggle-section-label">&#9776;&nbsp;&nbsp;CMX</label><input type="checkbox" id="nav-toggle-cmx" class="hidden toggle-section-button" checked="checked">
                        <section class="toggle-section">

                            <div class="input-group">
                                <label for="cmx-precision">CMX Precision</label><select id="cmx-precision" class="reg-action" reg="0x5380" offset="1" mask="0x01">
                                    <option value="1">2.6 Mode</option>
                                    <option value="0" selected="selected">1.7 Mode</option>
                                </select>
                            </div>
                            <div class="input-group"><label for="cmx1">CMX1 for Y</label><div class="switch"><input id="cmx1" type="checkbox" class="reg-action" reg="0x5381" offset="1" mask="0x01"><label class="slider" for="cmx1"></label></div></div>
                            <div class="input-group"><label for="cmx11">CMX1 for Y Neg</label><div class="switch"><input id="cmx11" type="checkbox" class="reg-action" reg="0x538b" offset="0" mask="0x01"><label class="slider" for="cmx11"></label></div></div>

                            <div class="input-group"><label for="cmx2">CMX2 for Y</label><input type="range" id="cmx2" min="0" max="255" value="0" class="reg-action" reg="0x5382" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="cmx12">CMX2 for Y Neg</label><div class="switch"><input id="cmx12" type="checkbox" class="reg-action" reg="0x538b" offset="1" mask="0x01"><label class="slider" for="cmx12"></label></div></div>
                            <div class="input-group"><label for="cmx3">CMX3 for Y</label><input type="range" id="cmx3" min="0" max="255" value="0" class="reg-action" reg="0x5383" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="cmx13">CMX3 for Y Neg</label><div class="switch"><input id="cmx13" type="checkbox" class="reg-action" reg="0x538b" offset="2" mask="0x01"><label class="slider" for="cmx13"></label></div></div>
                            <div class="input-group"><label for="cmx4">CMX4 for Y</label><input type="range" id="cmx4" min="0" max="255" value="0" class="reg-action" reg="0x5384" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="cmx14">CMX4 for Y Neg</label><div class="switch"><input id="cmx14" type="checkbox" class="reg-action" reg="0x538b" offset="3" mask="0x01"><label class="slider" for="cmx14"></label></div></div>
                            <div class="input-group"><label for="cmx5">CMX5 for Y</label><input type="range" id="cmx5" min="0" max="255" value="0" class="reg-action" reg="0x5385" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="cmx15">CMX5 for Y Neg</label><div class="switch"><input id="cmx15" type="checkbox" class="reg-action" reg="0x538b" offset="4" mask="0x01"><label class="slider" for="cmx15"></label></div></div>
                            <div class="input-group"><label for="cmx6">CMX6 for Y</label><input type="range" id="cmx6" min="0" max="255" value="0" class="reg-action" reg="0x5386" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="cmx16">CMX6 for Y Neg</label><div class="switch"><input id="cmx16" type="checkbox" class="reg-action" reg="0x538b" offset="5" mask="0x01"><label class="slider" for="cmx16"></label></div></div>
                            <div class="input-group"><label for="cmx7">CMX7 for Y</label><input type="range" id="cmx7" min="0" max="255" value="0" class="reg-action" reg="0x5387" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="cmx17">CMX7 for Y Neg</label><div class="switch"><input id="cmx17" type="checkbox" class="reg-action" reg="0x538b" offset="6" mask="0x01"><label class="slider" for="cmx17"></label></div></div>
                            <div class="input-group"><label for="cmx8">CMX8 for Y</label><input type="range" id="cmx8" min="0" max="255" value="0" class="reg-action" reg="0x5388" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="cmx18">CMX8 for Y Neg</label><div class="switch"><input id="cmx18" type="checkbox" class="reg-action" reg="0x538b" offset="7" mask="0x01"><label class="slider" for="cmx18"></label></div></div>
                            <div class="input-group"><label for="cmx9">CMX9 for Y</label><input type="range" id="cmx9" min="0" max="255" value="0" class="reg-action" reg="0x5389" offset="0" mask="0xff"></div>
                            <div class="input-group"><label for="cmx10">CMX9 for Y Neg</label><div class="switch"><input id="cmx10" type="checkbox" class="reg-action" reg="0x538a" offset="0" mask="0x01"><label class="slider" for="cmx10"></label></div></div>

                        </section>
                        <hr style="width:100%">
                        <label for="nav-toggle-awbg" class="toggle-section-label">&#9776;&nbsp;&nbsp;AWB Gain</label><input type="checkbox" id="nav-toggle-awbg" class="hidden toggle-section-button" checked="checked">
                        <section class="toggle-section">

                            <div class="input-group"><label for="awbg1">Manual AWB Gain</label><div class="switch"><input id="awbg1" type="checkbox" class="reg-action" reg="0x3406" offset="0" mask="0x01"><label class="slider" for="awbg1"></label></div></div>
                            <div class="input-group"><label for="awbg2">AWB R Gain</label><input type="range" id="awbg2" min="0" max="4095" value="0" class="reg-action" reg="0x3400" offset="0" mask="0xfff"></div>
                            <div class="input-group"><label for="awbg3">AWB G Gain</label><input type="range" id="awbg3" min="0" max="4095" value="0" class="reg-action" reg="0x3402" offset="0" mask="0xfff"></div>
                            <div class="input-group"><label for="awbg4">AWB B Gain</label><input type="range" id="awbg4" min="0" max="4095" value="0" class="reg-action" reg="0x3404" offset="0" mask="0xfff"></div>

                        </section>
                        <hr style="width:100%">
                        <label for="nav-toggle-aecagc" class="toggle-section-label">&#9776;&nbsp;&nbsp;AEC/AGC Control</label><input type="checkbox" id="nav-toggle-aecagc" class="hidden toggle-section-button" checked="checked">
                        <section class="toggle-section">

                            <div class="input-group"><label for="aecagc1">Manual AGC</label><div class="switch"><input id="aecagc1" type="checkbox" class="reg-action" reg="0x3503" offset="1" mask="0x01"><label class="slider" for="aecagc1"></label></div></div>
                            <div class="input-group"><label for="aecagc2">Manual AEC</label><div class="switch"><input id="aecagc2" type="checkbox" class="reg-action" reg="0x3503" offset="0" mask="0x01"><label class="slider" for="aecagc2"></label></div></div>
                            <div class="input-group"><label for="aecagc3">AEC PK Exposure</label><input type="range" id="aecagc3" min="0" max="65535" value="0" class="reg-action" reg="0x3500" offset="0" mask="0xffff0"></div>
                            <div class="input-group"><label for="aecagc4">AEC PK Real Gain</label><input type="range" id="aecagc4" min="0" max="1023" value="0" class="reg-action" reg="0x350a" offset="0" mask="0x3ff"></div>
                            <div class="input-group"><label for="aecagc5">AEC PK VTS</label><input type="range" id="aecagc5" min="0" max="65535" value="0" class="reg-action" reg="0x350c" offset="0" mask="0xffff"></div>

                        </section>


                    </nav>
                </div>
                <figure>
                    <div id="stream-container" class="image-container hidden">
                        <a id="save-still" href="#" class="button save" download="capture.jpg">Save</a>
                        <div class="close" id="close-stream">Ã</div>
                        <img id="stream" src="" crossorigin>
                    </div>
                </figure>
            </div>
        </section>
        <script>
document.addEventListener('DOMContentLoaded', function (event) {
  var baseHost = document.location.origin
  var streamUrl = baseHost + ':81'

  function fetchUrl(url, cb){
    fetch(url)
      .then(function (response) {
        if (response.status !== 200) {
          cb(response.status, response.statusText);
        } else {
          response.text().then(function(data){
            cb(200, data);
          }).catch(function(err) {
            cb(-1, err);
          });
        }
      })
      .catch(function(err) {
        cb(-1, err);
      });
  }

  function setReg(reg, offset, mask, value, cb){
    //console.log('Set Reg', '0x'+reg.toString(16), offset, '0x'+mask.toString(16), '0x'+value.toString(16), '('+value+')');
    value = (value & mask) << offset;
    mask = mask << offset;
    fetchUrl(`${baseHost}/reg?reg=${reg}&mask=${mask}&val=${value}`, cb);
  }

  function getReg(reg, offset, mask, cb){
    mask = mask << offset;
    fetchUrl(`${baseHost}/greg?reg=${reg}&mask=${mask}`, function(code, txt){
      let value = 0;
      if(code == 200){
        value = parseInt(txt);
        value = (value & mask) >> offset;
        txt = ''+value;
      }
      //console.log('Get Reg', '0x'+reg.toString(16), offset, '0x'+(mask >> offset).toString(16), '0x'+value.toString(16), '('+txt+')');
      cb(code, txt);
    });
  }

  function setXclk(xclk, cb){
    fetchUrl(`${baseHost}/xclk?xclk=${xclk}`, cb);
  }

  function setPll(bypass, mul, sys, root_, pre, seld5, pclken, pclk, cb){
    fetchUrl(`${baseHost}/pll?bypass=${bypass}&mul=${mul}&sys=${sys}&root=${root_}&pre=${pre}&seld5=${seld5}&pclken=${pclken}&pclk=${pclk}`, cb);
  }

  function setWindow(start_x, start_y, end_x, end_y, offset_x, offset_y, total_x, total_y, output_x, output_y, scaling, binning, cb){
    fetchUrl(`${baseHost}/resolution?sx=${start_x}&sy=${start_y}&ex=${end_x}&ey=${end_y}&offx=${offset_x}&offy=${offset_y}&tx=${total_x}&ty=${total_y}&ox=${output_x}&oy=${output_y}&scale=${scaling}&binning=${binning}`, cb);
  }


  const setRegValue = (el) => {
    let reg = el.attributes.reg?parseInt(el.attributes.reg.nodeValue):0;
    let offset = el.attributes.offset?parseInt(el.attributes.offset.nodeValue):0;
    let mask = el.attributes.mask?parseInt(el.attributes.mask.nodeValue):255;
    let value = 0;
    switch (el.type) {
      case 'checkbox':
        value = el.checked ? mask : 0;
        break;
      case 'range':
      case 'text':
      case 'select-one':
        value = el.value;
        break
      default:
        return;
    }

    setReg(reg, offset, mask, value, function(code, txt){
      if(code != 200){
        alert('Error['+code+']: '+txt);
      }
    });
  }

  // Attach on change action for register elements
  document
    .querySelectorAll('.reg-action')
    .forEach(el => {
        if (el.type === 'text') {
            el.onkeyup = function(e){
                if(e.keyCode == 13){
                    setRegValue(el);
                }
            }
        } else {
            el.onchange = () => setRegValue(el)
        }
    })


  const updateRegValue = (el, value, updateRemote) => {
    let initialValue;
    let offset = el.attributes.offset?parseInt(el.attributes.offset.nodeValue):0;
    let mask = (el.attributes.mask?parseInt(el.attributes.mask.nodeValue):255) << offset;
    value = (value & mask) >> offset;
    if (el.type === 'checkbox') {
      initialValue = el.checked
      value = !!value
      el.checked = value
    } else {
      initialValue = el.value
      el.value = value
    }
  }


  const printReg = (el) => {
    let reg = el.attributes.reg?parseInt(el.attributes.reg.nodeValue):0;
    let offset = el.attributes.offset?parseInt(el.attributes.offset.nodeValue):0;
    let mask = el.attributes.mask?parseInt(el.attributes.mask.nodeValue):255;
    let value = 0;
    switch (el.type) {
      case 'checkbox':
        value = el.checked ? mask : 0;
        break;
      case 'range':
      case 'select-one':
        value = el.value;
        break
      default:
        return;
    }
    value = (value & mask) << offset;
    return '0x'+reg.toString(16)+', 0x'+value.toString(16);
  }

  document.getElementById('get-gamma').onclick = () => {
    let out = 'static const DRAM_ATTR uint16_t sensor_regs_gamma0[][2] = {\n';
    out += '    {'+printReg(document.getElementById('gamma-bias'))+'},\n'
    for (var i = 1; i < 17; i++) {
      out += '    {'+printReg(document.getElementById('gamma-'+i))+'}'+((i<16)?',':'')+'\n'
    }
    out += '};\n';
    alert(out);
  }

  const setRegButton = document.getElementById('set-reg')
  setRegButton.onclick = () => {
    let reg = parseInt(document.getElementById('reg-addr').value);
    let mask = parseInt(document.getElementById('reg-mask').value);
    let value = parseInt(document.getElementById('reg-value').value);

    setReg(reg, 0, mask, value, function(code, txt){
      if(code != 200){
        alert('Error['+code+']: '+txt);
      }
    });
  }

  const getRegButton = document.getElementById('get-reg')
  getRegButton.onclick = () => {
    let reg = parseInt(document.getElementById('get-reg-addr').value);
    let mask = parseInt(document.getElementById('get-reg-mask').value);
    let value = document.getElementById('get-reg-value');

    getReg(reg, 0, mask, function(code, txt){
      if(code != 200){
        value.innerHTML = 'Error['+code+']: '+txt;
      } else {
        value.innerHTML = '0x'+parseInt(txt).toString(16)+' ('+txt+')';
      }
    });
  }

  const setXclkButton = document.getElementById('set-xclk')
  setXclkButton.onclick = () => {
    let xclk = parseInt(document.getElementById('xclk').value);

    setXclk(xclk, function(code, txt){
      if(code != 200){
        alert('Error['+code+']: '+txt);
      }
    });
  }

  const setResButton = document.getElementById('set-resolution')
  setResButton.onclick = () => {
    let start_x = parseInt(document.getElementById('start-x').value);
    let start_y = parseInt(document.getElementById('start-y').value);
    let end_x = parseInt(document.getElementById('end-x').value);
    let end_y = parseInt(document.getElementById('end-y').value);
    let offset_x = parseInt(document.getElementById('offset-x').value);
    let offset_y = parseInt(document.getElementById('offset-y').value);
    let total_x = parseInt(document.getElementById('total-x').value);
    let total_y = parseInt(document.getElementById('total-y').value);
    let output_x = parseInt(document.getElementById('output-x').value);
    let output_y = parseInt(document.getElementById('output-y').value);
    let scaling = document.getElementById('scaling').checked?1:0;
    let binning = document.getElementById('binning').checked?1:0;

    setWindow(start_x, start_y, end_x, end_y, offset_x, offset_y, total_x, total_y, output_x, output_y, scaling, binning, function(code, txt){
      if(code != 200){
        alert('Error['+code+']: '+txt);
      }
    });
  }

  const setPllButton = document.getElementById('set-pll')
  setPllButton.onclick = () => {
    var bypass = document.getElementById('bypass-pll').checked?1:0;
    var mul = parseInt(document.getElementById('mul-pll').value);
    var sys = parseInt(document.getElementById('sys-pll').value);
    var root_ = parseInt(document.getElementById('root-pll').value);
    var pre = parseInt(document.getElementById('pre-pll').value);
    var seld5 = parseInt(document.getElementById('seld5-pll').value);
    var pclken = document.getElementById('pclk-en').checked?1:0;
    var pclk = parseInt(document.getElementById('pclk-pll').value);

    setPll(bypass, mul, sys, root_, pre, seld5, pclken, pclk, function(code, txt){
      if(code != 200){
        alert('Error['+code+']: '+txt);
      }
    });
  }

  const saveButton = document.getElementById('save-still');

  saveButton.onclick = () => {
    var canvas = document.createElement("canvas");
    canvas.width = view.width;
    canvas.height = view.height;
    document.body.appendChild(canvas);
    var context = canvas.getContext('2d');
    context.drawImage(view,0,0);
    try {
      var dataURL = canvas.toDataURL('image/jpeg');
      saveButton.href = dataURL;
      var d = new Date();
      saveButton.download = d.getFullYear() + ("0"+(d.getMonth()+1)).slice(-2) + ("0" + d.getDate()).slice(-2) + ("0" + d.getHours()).slice(-2) + ("0" + d.getMinutes()).slice(-2) + ("0" + d.getSeconds()).slice(-2) + ".jpg";
    } catch (e) {
      console.error(e);
    }
    canvas.parentNode.removeChild(canvas);
  }
  
  const hide = el => {
    el.classList.add('hidden')
  }
  const show = el => {
    el.classList.remove('hidden')
  }

  const disable = el => {
    el.classList.add('disabled')
    el.disabled = true
  }

  const enable = el => {
    el.classList.remove('disabled')
    el.disabled = false
  }

  const updateValue = (el, value, updateRemote) => {
    updateRemote = updateRemote == null ? true : updateRemote
    let initialValue
    if (el.type === 'checkbox') {
      initialValue = el.checked
      value = !!value
      el.checked = value
    } else {
      initialValue = el.value
      el.value = value
    }

    if (updateRemote && initialValue !== value) {
      updateConfig(el);
    } else if(!updateRemote){
      if(el.id === "aec"){
        value ? hide(exposure) : show(exposure)
      } else if(el.id === "agc"){
        if (value) {
          //show(gainCeiling)
          hide(agcGain)
        } else {
          //hide(gainCeiling)
          show(agcGain)
        }
      } else if(el.id === "awb_gain"){
        value ? show(wb) : hide(wb)
      } else if(el.id === "face_recognize"){
        value ? enable(enrollButton) : disable(enrollButton)
      }
    }
  }

  function updateConfig (el) {
    let value
    switch (el.type) {
      case 'checkbox':
        value = el.checked ? 1 : 0
        break
      case 'range':
      case 'select-one':
        value = el.value
        break
      case 'button':
      case 'submit':
        value = '1'
        break
      default:
        return
    }

    const query = `${baseHost}/control?var=${el.id}&val=${value}`

    fetch(query)
      .then(response => {
        console.log(`request to ${query} finished, status: ${response.status}`)
      })
  }

  // Attach default on change action
  document
    .querySelectorAll('.default-action')
    .forEach(el => {
      el.onchange = () => updateConfig(el)
    })

  document
    .querySelectorAll('.close')
    .forEach(el => {
      el.onclick = () => {
        hide(el.parentNode)
      }
    })

  // read initial values
  fetch(`${baseHost}/status`)
    .then(function (response) {
      return response.json()
    })
    .then(function (state) {
      document
        .querySelectorAll('.default-action')
        .forEach(el => {
            updateValue(el, state[el.id], false)
        })
      document
        .querySelectorAll('.reg-action')
        .forEach(el => {
            let reg = el.attributes.reg?parseInt(el.attributes.reg.nodeValue):0;
            if(reg == 0){
              return;
            }
            updateRegValue(el, state['0x'+reg.toString(16)], false)
        })
    })

  const view = document.getElementById('stream')
  const viewContainer = document.getElementById('stream-container')
  const stillButton = document.getElementById('get-still')
  const streamButton = document.getElementById('toggle-stream')
  const enrollButton = document.getElementById('face_enroll')
  const closeButton = document.getElementById('close-stream')

  const stopStream = () => {
    window.stop();
    streamButton.innerHTML = 'Start Stream'
  }

  const startStream = () => {
    view.src = `${streamUrl}/stream`
    show(viewContainer)
    streamButton.innerHTML = 'Stop Stream'
  }

  // Attach actions to buttons
  stillButton.onclick = () => {
    stopStream()
    view.src = `${baseHost}/capture?_cb=${Date.now()}`
    show(viewContainer)
  }

  closeButton.onclick = () => {
    stopStream()
    hide(viewContainer)
  }

  streamButton.onclick = () => {
    const streamEnabled = streamButton.innerHTML === 'Stop Stream'
    if (streamEnabled) {
      stopStream()
    } else {
      startStream()
    }
  }

  enrollButton.onclick = () => {
    updateConfig(enrollButton)
  }

  // Custom actions
  // Gain
  const agc = document.getElementById('agc')
  const agcGain = document.getElementById('agc_gain-group')
  //const gainCeiling = document.getElementById('gainceiling-group')
  agc.onchange = () => {
    updateConfig(agc)
    if (agc.checked) {
      //show(gainCeiling)
      hide(agcGain)
    } else {
      //hide(gainCeiling)
      show(agcGain)
    }
  }

  // Exposure
  const aec = document.getElementById('aec')
  const exposure = document.getElementById('aec_value-group')
  aec.onchange = () => {
    updateConfig(aec)
    aec.checked ? hide(exposure) : show(exposure)
  }

  // AWB
  const awb = document.getElementById('awb_gain')
  const wb = document.getElementById('wb_mode-group')
  awb.onchange = () => {
    updateConfig(awb)
    awb.checked ? show(wb) : hide(wb)
  }

  // Detection and framesize
  const detect = document.getElementById('face_detect')
  const recognize = document.getElementById('face_recognize')
  const framesize = document.getElementById('framesize')

  framesize.onchange = () => {
    updateConfig(framesize)
    if (framesize.value > 5) {
      updateValue(detect, false)
      updateValue(recognize, false)
    }
  }

  detect.onchange = () => {
    if (framesize.value > 5) {
      alert("Please select CIF or lower resolution before enabling this feature!");
      updateValue(detect, false)
      return;
    }
    updateConfig(detect)
    if (!detect.checked) {
      disable(enrollButton)
      updateValue(recognize, false)
    }
  }

  recognize.onchange = () => {
    if (framesize.value > 5) {
      alert("Please select CIF or lower resolution before enabling this feature!");
      updateValue(recognize, false)
      return;
    }
    updateConfig(recognize)
    if (recognize.checked) {
      enable(enrollButton)
      updateValue(detect, true)
    } else {
      disable(enrollButton)
    }
  }
})

        </script>
    </body>
</html>
"""

# Convert the input and save to a file
result = convert_to_quoted_lines(html_code)
output_filename = "formatted_html.txt"
save_to_file(result, output_filename)

print(f"Output saved to {output_filename}")