
/* Section 1: Interactive editing demo */
let fg_options = {
    'car-turn' : [
        'source',
        'sports',
        'minivan',
        'SUV',
    ],
    'blackswan': [
        'source',
        'duck',
        'whiteswan',
    ],
    'boat': [
        'source',
        'yacht',
    ],
    'libby': [
        'source',
        'cat1',
        'cat2',
    ],
    'surf': [
        'source',
        'sailboat',
    ],
    'motorbike': [
        'source',
        'bears',
    ],
    'kite-surf': [
        'source',
        'monkey',
    ],
    'truck': [
        'source',
        'train',
    ],
    'kid': [
        'source',
        'robot',
    ],
};

let bg_options = {
    'car-turn' : [
        'source',
        'grassland',
        'snow',
        'rural',
    ],
    'blackswan': [
        'source',
        'cartoon-style',
    ],
    'boat': [
        'source',
        'ice-river',
        'cartoon-style',
    ],
    'libby': [
        'source',
        'cosmos',
    ],
    'surf': [
        'source',
        'cartoon-style',
    ],
    'motorbike': [
        'source',
        'polar-ice',
    ],
    'kite-surf': [
        'source',
        'river',
    ],
    'truck': [
        'source',
        'railway',
    ],
    'kid': [
        'source',
        'polluted-beach',
    ],
};

var selected_video = 'car-turn';
var selected_fg = 'sports';
var selected_bg = 'grassland';
var video1_path = '';
var video2_path = '';

var prev_video1_path = 'none';
var prev_video2_path = 'none';

function loadVideo(is_end) {
    var video1 = document.getElementById('video-player1');
    var video2 = document.getElementById('video-player2');
    var video_src1 = document.getElementById('video-src1');
    var video_src2 = document.getElementById('video-src2');

    var width = document.getElementById('container').clientWidth;
    var height = width / 768 * 432;
    video1.style.width = '' + width + 'px';
    video2.style.width = '' + width + 'px';
    video1.style.height = '' + height + 'px';
    video2.style.height = '' + height + 'px';
    
    video1.pause();
    video2.pause();
    if(prev_video1_path != video1_path) {
        video_src1.setAttribute('src', video1_path);
        video1.load();
    }
    if(prev_video2_path != video2_path) {
        video_src2.setAttribute('src', video2_path);
        video2.load();
    }
    prev_video1_path = video1_path;
    prev_video2_path = video2_path;
    
    video1.currentTime = 0;
    video2.currentTime = 0;
    
    video1.playbackRate = 1.6;
    video2.playbackRate = 1.6;
    
    video1.play();
    video2.play();        
}

function selectVideo(video) {
    selected_video = video;
    selected_fg = fg_options[video][1];
    selected_bg = bg_options[video][1];
    showSelected();
    load_options(video);
    update_video_source();
}


function selectFG(index) {
    selected_fg = fg_options[selected_video][index];
    showSelected();
    update_video_source();
}

function selectBG(index) {
    selected_bg = bg_options[selected_video][index];
    showSelected();
    update_video_source();
}

function update_video_source() {
    video1_path = 'videos/' + selected_video + '/input.mp4';
    video2_path = 'videos/' + selected_video + '/' + selected_fg + '_' + selected_bg + '.mp4';
    console.log(video2_path);

    loadVideo();
}

function load_options(video) {
    // foreground
    fgs = fg_options[video];
    bgs = bg_options[video];

    fg_btns = document.getElementsByClassName('btn-fg');
    for(var i = 0; i < fg_btns.length; i++) {
        if(i < fgs.length) {
            fg_btns[i].style.display = 'initial';
            fg_btns[i].innerHTML = fgs[i];
        } else {
            fg_btns[i].style.display = 'none';
        }
    }

    bg_btns = document.getElementsByClassName('btn-bg');
    for(var i = 0; i < bg_btns.length; i++) {
        if(i < bgs.length) {
            bg_btns[i].style.display = 'initial';
            bg_btns[i].innerHTML = bgs[i];
        } else {
            bg_btns[i].style.display = 'none';
        }
    }
}


function set_inactive(btn) {
    btn.classList.remove('on');
}
function set_active(btn) {
    btn.classList.add('on');
}

function showSelected() {
    var video_btns = document.getElementsByClassName('btn-video');
    for(var i = 0; i < video_btns.length; i++) {
        set_inactive(video_btns[i]);
    }
    selected_index = Object.keys(fg_options).indexOf(selected_video);
    set_active(video_btns[selected_index]);

    var fg_btns = document.getElementsByClassName('btn-fg');
    for(var i = 0; i < fg_btns.length; i++) {
        set_inactive(fg_btns[i]);
    }
    selected_index = fg_options[selected_video].indexOf(selected_fg);
    set_active(fg_btns[selected_index]);

    var bg_btns = document.getElementsByClassName('btn-bg');
    for(var i = 0; i < bg_btns.length; i++) {
        set_inactive(bg_btns[i]);
    }
    selected_index = bg_options[selected_video].indexOf(selected_bg);
    set_active(bg_btns[selected_index]);


    // section 2: comparison

    var comp_video_btns = document.getElementsByClassName('btn-comp-video');
    for(var i = 0; i < comp_video_btns.length; i++) {
        set_inactive(comp_video_btns[i]);
    }
    selected_index = ['car-turn', 'blackswan', 'boat', 'libby'].indexOf(selected_compare_video);
    set_active(comp_video_btns[selected_index]);

    var comp_method_btns = document.getElementsByClassName('btn-comp-method');
    for(var i = 0; i < comp_method_btns.length; i++) {
        set_inactive(comp_method_btns[i]);
    }
    selected_index = ['input', 'multi-frame', 'single-frame', 'text2live'].indexOf(selected_compare_method);
    set_active(comp_method_btns[selected_index]);

    // section 3: interpolation
    var app_btns = document.getElementsByClassName('btn-app-video');
    for(var i = 0; i < app_btns.length; i++) {
        set_inactive(app_btns[i]);
    }
    selected_index = ['duck', 'whiteswan', 'sports', 'motorbike'].indexOf(selected_app_video);
    set_active(app_btns[selected_index]);
}

/* Section 2: Visual comparison with baseline methods and SOTA */

var selected_compare_video = 'car-turn';
var selected_compare_method = 'text2live';
var comp1_path = '';
var comp2_path = '';

function selectComparisonVideo(video) {
    selected_compare_video = video;
    update_comparison_source();
    showSelected();
}

function selectComparedMethod(method) {
    selected_compare_method = method;
    update_comparison_source();
    showSelected();
}

let method_full_names = {
    'input': 'Input',
    'multi-frame': 'Multi-frame editing + frame interpolation<br/>(FILM [Reda et al. ECCV 2022])',
    'single-frame': 'Single-frame editing + frame propagation<br/>(EbSynth [Jamriška et al. TOG 2019])',
    'text2live': 'Text2LIVE [Bar-Tal et al. ECCV 2022]',
};

function update_comparison_source() {
    comp1_path = 'videos/' + selected_compare_video + '/' + selected_compare_method + '.mp4';
    comp2_path = 'videos/' + selected_compare_video + '/ours.mp4';

    loadComparison(false);
    document.getElementById('comparison-caption').innerHTML = method_full_names[selected_compare_method];
}

var prev_comp1_path = 'none';
var prev_comp2_path = 'none';
function loadComparison() {
    var video1 = document.getElementById('comparison-player1');
    var video2 = document.getElementById('comparison-player2');
    var video_src1 = document.getElementById('comparison-src1');
    var video_src2 = document.getElementById('comparison-src2');
    
    var width = document.getElementById('container').clientWidth;
    document.getElementById('comparison-video-div').style.height = '' + (width / 768 * 432 / 2) + 'px';
    
    video1.pause();
    video2.pause();
    if(prev_comp1_path != comp1_path) {
        video_src1.setAttribute('src', comp1_path);
        video1.load();
    }
    if(prev_comp2_path != comp2_path) {
        video_src2.setAttribute('src', comp2_path);
        video2.load();
    }
    prev_comp1_path = comp1_path;
    prev_comp2_path = comp2_path;

    video1.currentTime = 0;
    video2.currentTime = 0;

    video1.playbackRate = 1.6;
    video2.playbackRate = 1.6;
    video1.play();
    video2.play();
}

/* Section 3: Application: Shape-aware interpolation */

var selected_app_video = 'whiteswan';
var app_path = '';
function selectAppVideo(video) {
    selected_app_video = video;
    update_app_source();
    showSelected();
}

function update_app_source() {
    app_path = 'videos/interpolation/' + selected_app_video + '.mp4';
    loadApp();
}

var prev_app_path = "none";
function loadApp(is_end) {
    var video = document.getElementById('app-player');
    
    var video_src = document.getElementById('app-src');

    var width =document.getElementById('container').clientWidth;
    var height = width / 768 * 432;

    video.style.width = '' + width + 'px';
    video.style.height = '' + height + 'px';


    video.pause();
    if(prev_app_path != app_path) {
        video_src.setAttribute('src', app_path);
        video.load();
    }
    prev_app_path = app_path;
    video.currentTime = 0;
    video.playbackRate = 1.8;
    video.play();
}
