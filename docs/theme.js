// SPDX-License-Identifier: MIT
// theme.js: light and dark for every page of the site. Load it as a plain (blocking) script at the top of <head>, so the
// page never paints in the wrong theme first.
//
// The page follows the system (or the app showing it, which may set data-theme on <html>) until the visitor presses a
// [data-theme-toggle] button. That pick is kept in this browser (localStorage; when storage is blocked it lasts for the
// visit) and wins on every page of the site.
//
// For page code: theme.dark() says which theme shows now. A "themechange" event ({ detail: { dark } }) fires on window
// whenever it changes, from a press, the system or the app. A press runs inside a View Transition, so the whole page,
// 3D canvases included, crossfades: a listener that redraws a canvas must draw right away, not in the next frame.
(function () {
  var KEY = "takt-theme", root = document.documentElement;
  var mq = window.matchMedia ? matchMedia("(prefers-color-scheme: dark)") : { matches: false };
  var saved = null;
  try { saved = localStorage.getItem(KEY); } catch (e) { /* storage blocked */ }
  if (saved === "dark" || saved === "light") root.setAttribute("data-theme", saved);

  function dark() { var t = root.getAttribute("data-theme"); return t === "dark" || t === "light" ? t === "dark" : !!mq.matches; }
  var last = dark();
  function changed() {
    var d = dark();
    if (d === last) return;
    last = d;
    sync();
    dispatchEvent(new CustomEvent("themechange", { detail: { dark: d } }));
  }
  if (mq.addEventListener) mq.addEventListener("change", changed); else if (mq.addListener) mq.addListener(changed);
  if (window.MutationObserver) new MutationObserver(changed).observe(root, { attributes: true, attributeFilter: ["data-theme"] });

  function set(d) {
    root.setAttribute("data-theme", d ? "dark" : "light");
    try { localStorage.setItem(KEY, d ? "dark" : "light"); } catch (e) { /* storage blocked: this visit only */ }
    changed();
  }
  function toggle() {
    var d = !dark(), still = window.matchMedia && matchMedia("(prefers-reduced-motion: reduce)").matches;
    if (!document.startViewTransition || still || document.hidden) { set(d); return; }
    try {
      var vt = document.startViewTransition(function () { set(d); });
      if ("liveVT" in window) window.liveVT = vt;   // bom.html: a tap during the crossfade still reaches its button
      vt.ready.catch(function () {}); vt.updateCallbackDone.catch(function () {}); vt.finished.catch(function () {});
    } catch (e) { set(d); }
  }

  // every toggle on the page: aria-pressed is true while dark shows, the label names what a press does
  function sync() {
    var d = dark(), bs = document.querySelectorAll("[data-theme-toggle]");
    for (var i = 0; i < bs.length; i++) {
      var b = bs[i];
      b.setAttribute("aria-pressed", String(d));
      b.setAttribute("aria-label", d ? "Switch to light mode" : "Switch to dark mode");
      b.title = d ? "Light mode" : "Dark mode";
    }
  }
  function wire() {
    var bs = document.querySelectorAll("[data-theme-toggle]");
    for (var i = 0; i < bs.length; i++) if (!bs[i].themeWired) { bs[i].themeWired = true; bs[i].addEventListener("click", toggle); }
    sync();
  }
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", wire); else wire();

  window.theme = { dark: dark, toggle: toggle, set: set };
})();
