var copyButton = document.getElementById("copyButton");
    
copyButton.addEventListener("click", function () {
  var codeElement = document.getElementById("bibtexCode");
  var textToCopy = codeElement.textContent;

  var textarea = document.createElement("textarea");
  textarea.value = textToCopy;
  document.body.appendChild(textarea);
  textarea.select();
  document.execCommand("copy");
  document.body.removeChild(textarea);

  copyButton.textContent = "✓ Copied!";
  copyButton.style.backgroundColor = "#8dd3c7";

  setTimeout(function () {
    copyButton.textContent = "Copy";
    copyButton.style.backgroundColor = "#ffffff";
  }, 2000); // Reset the button text after 2 seconds (adjust as needed)
});

$(document).ready(function(){
    var zoom = mediumZoom('[data-zoomable]', {
        background: 'rgba(0, 0, 0, 0.5)' // 50% alpha transparency
    });

    zoom.on('open', () => {
        copyButton.style.zIndex = '-1';
    });

    zoom.on('close', () => {
        copyButton.style.zIndex = '1';
    });
});
