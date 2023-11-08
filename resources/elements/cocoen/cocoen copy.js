var Cocoen=(()=>{var g=Object.defineProperty;var b=(n,e)=>{for(var t in e)g(n,t,{get:e[t],enumerable:!0})};var f={};b(f,{create:()=>s,parse:()=>p});var a="cocoen",r=`${a}-component`;var l=n=>Number.parseInt(window.getComputedStyle(n).width,10);var h=({dragElementWidth:n,hostElementWidth:e,x:t})=>{let i=t;t<0?i=n:t>=e&&(i=e-n);let o=i+n/2;return o/=e,o*100};var d=(n,e)=>{let t=0;n instanceof MouseEvent?t=n.clientX:n instanceof TouchEvent&&(t=n.touches[0].clientX);let i = e ? e.getBoundingClientRect().left : 0;return t-i};var c=(n,e)=>{let t;return(...i)=>{let o;return t&&clearTimeout(t),t=setTimeout(()=>{o=n(...i)},e),o}};var m=n=>`${n}%`;var E=`
  :host {
    box-sizing: border-box;
    -moz-box-sizing: border-box;
    -webkit-box-sizing: border-box;
    cursor: pointer;
    display: block;
    overflow: hidden;
    position: relative;
    user-select: none;
    -webkit-user-select: none;
    -moz-user-select: none;
    -khtml-user-select: none;
    -ms-user-select: none;
  }

  :host *,
  :host *:after,
  :host *:before {
    box-sizing: inherit;
    -moz-box-sizing: inherit;
    -webkit-box-sizing: inherit;
  }

  #before {
    height: 100%;
    left: 0;
    overflow: hidden;
    position: absolute;
    top: 0;
    width: 50%;
  }

  #drag {
    background: var(--color, #fff);
    bottom: 0;
    cursor: ew-resize;
    left: 50%;
    margin-left: -1px;
    position: absolute;
    top: 0;
    width: 2px;
  }

  #drag:before {
    border: 3px solid var(--color, #fff);
    content: '';
    height: 30px;
    left: 50%;
    margin-left: -7px;
    margin-top: -15px;
    position: absolute;
    top: 50%;
    width: 14px;
  }

  ::slotted(video) {
    max-height: 100%;
    object-fit: contain;
    pointer-events: none;
  }

  ::slotted(video[slot=before]) {
    max-width: none;
  }

  ::slotted(video[slot=after]) {
    display: block;
    max-width: 100%;
    width: 100%;
  }
`,v=class extends HTMLElement{constructor(){super();this.animateToValue=0;this.colorValue="#fff";this.dragElementWidthValue=0;this.elementWidthValue=0;this.isDraggingValue=!1;this.openRatioValue=50;this.isRenderedValue=!1;this.isVisibleValue=!1;this.xValue=0;this.drag=null,this.shadowDOM=this.attachShadow({mode:"open"}),this.onDragStartHandler=()=>this.onDragStart(),this.onDragEndHandler=()=>this.onDragEnd(),this.onDragHandler=e=>this.onDrag(e),this.onClickHandler=e=>this.onClick(e),this.onContextMenuHandler=()=>this.onContextMenu(),this.onIntersectionHandler=(e,t)=>this.onIntersection(e,t),this.debouncedUpdateDimensions=c(()=>{this.updateDimensions()},250),this.intersectionObserver=new IntersectionObserver(this.onIntersectionHandler,{root:null,rootMargin:"0px",threshold:0})}get x(){return this.xValue}set x(e){this.xValue=e,window.requestAnimationFrame(()=>{this.openRatio=h({x:this.xValue,dragElementWidth:this.dragElementWidth,hostElementWidth:this.elementWidth})})}get elementWidth(){return this.elementWidthValue}set elementWidth(e){this.elementWidthValue=e}get dragElementWidth(){return this.dragElementWidthValue}set dragElementWidth(e){this.dragElementWidthValue=e}get isDragging(){return this.isDraggingValue}set isDragging(e){this.isDraggingValue=e}get openRatio(){return this.openRatioValue}set openRatio(e){this.openRatioValue=e,window.requestAnimationFrame(()=>{this.updateStyles()})}get color(){return this.colorValue}set color(e){this.colorValue=e,window.requestAnimationFrame(()=>{this.style.setProperty("--color",this.color)})}get isVisible(){return this.isVisibleValue}set isVisible(e){this.isVisibleValue=e}get isRendered(){return this.isRenderedValue}set isRendered(e){this.isRenderedValue=e}get animateTo(){return this.animateToValue}set animateTo(e){this.animateToValue=e}static get observedAttributes(){return["start","color"]}attributeChangedCallback(e,t,i){t!==i&&(e==="start"&&(this.animateTo=Number.parseInt(String(this.getAttribute("start")),10),this.isVisible&&(this.openRatio=this.animateTo)),e==="color"&&(this.color=String(this.getAttribute("color"))))}connectedCallback(){this.isRendered||(this.render(),this.isRendered=!0,this.dispatchEvent(new CustomEvent(`${r}:connected`,this.customEventPayload())),this.drag=this.shadowDOM.querySelector("#drag"),this.updateDimensions(),this.addEventListener("mouseover",this.onDragStartHandler,{passive:!0}),this.addEventListener("touchstart",this.onDragStartHandler,{passive:!0}),this.addEventListener("mousemove",this.onDragHandler,{passive:!0}),this.addEventListener("touchmove",this.onDragHandler,{passive:!0}),this.addEventListener("click",this.onClickHandler,{passive:!0}),this.addEventListener("contextmenu",this.onContextMenuHandler,{passive:!0}),window.addEventListener("resize",this.debouncedUpdateDimensions,{passive:!0}),window.addEventListener("mouseup",this.onDragEndHandler,{passive:!0}),window.addEventListener("touchend",this.onDragEndHandler,{passive:!0}),this.intersectionObserver.observe(this))}disconnectedCallback(){this.dispatchEvent(new CustomEvent(`${r}:disconnected`,this.customEventPayload())),this.removeEventListener("mousedown",this.onDragStartHandler),this.removeEventListener("touchstart",this.onDragStartHandler),this.removeEventListener("mousemove",this.onDragHandler),this.removeEventListener("touchmove",this.onDragHandler),this.removeEventListener("click",this.onClickHandler),this.removeEventListener("contextmenu",this.onContextMenuHandler),window.removeEventListener("resize",this.debouncedUpdateDimensions),window.removeEventListener("mouseup",this.onDragEndHandler),window.removeEventListener("touchend",this.onDragEndHandler),this.intersectionObserver.unobserve(this)}render(){this.shadowDOM.innerHTML=`
      <style>${E}</style>
      <div id="before">
        <slot name="before"'></slot>
      </div>
      <slot name="after"></slot>
      <div id="drag" part="drag"'></div>
    `}updateDimensions(){this.elementWidth=l(this),this.drag&&(this.dragElementWidth=l(this.drag)),this.dispatchEvent(new CustomEvent(`${r}:resized`,this.customEventPayload()))}updateStyles(){let e=this.shadowDOM.querySelector("#before"),t=this.shadowDOM.querySelector("#drag"),i=m(this.openRatio);this.animateTo?(e.style.transition="width .75s",t.style.transition="left .75s"):(e.style.transition="none",t.style.transition="none"),e.style.width=i,t.style.left=i,this.dispatchEvent(new CustomEvent(`${r}:updated`,this.customEventPayload()))}onDragStart(){this.animateTo=0,this.isDragging=!0}onDrag(e){!this.isDragging||(this.x=d(e,this))}onDragEnd(){this.isDragging=!1}onClick(e){this.animateTo=0,this.x=d(e,this)}onContextMenu(){this.isDragging=!1}onIntersection(e,t){e.forEach(i=>{i.isIntersecting&&(this.dispatchEvent(new CustomEvent(`${r}:visible`,this.customEventPayload())),this.animateTo&&(this.openRatio=this.animateTo),this.isVisible=!0,t.unobserve(this))})}customEventPayload(){return{bubbles:!0,composed:!0,detail:{elementWidth:this.elementWidth,openRatio:this.openRatio,isRendered:this.isRendered,isVisible:this.isVisible}}}};customElements.define(r,v);var s=(n,e)=>{let t=document.createElement(r),i=n.querySelectorAll("video")[0],o=n.querySelectorAll("video")[1];if(!i||!o)throw new Error("Cocoen needs two images");return i.setAttribute("slot","before"),o.setAttribute("slot","after"),t.append(i.cloneNode(!0)),t.append(o.cloneNode(!0)),Object.keys(n.dataset).forEach(u=>t.setAttribute(u,String(n.dataset[u]))),e?.start&&t.setAttribute("start",String(e.start)),e?.color&&t.setAttribute("color",e.color),n.replaceWith(t),t};var p=n=>{[...n.querySelectorAll(`.${a}`)].map(t=>s(t))};return f;})();
//# sourceMappingURL=cocoen.js.map
