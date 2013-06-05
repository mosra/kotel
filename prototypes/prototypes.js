var module = null;

function pageLoaded() {
    if(!module) setStatus('Loading...');
}

function moduleLoaded() {
    module = document.getElementById('module');
    setStatus('');
}

function setStatus(message) {
    var status = document.getElementById('status');
    if(status) status.innerHTML = message;
}

var listener = document.getElementById('listener');
listener.addEventListener('load', moduleLoaded, true);
