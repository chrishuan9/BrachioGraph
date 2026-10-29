/*
 * JavaScript port of BrachioGraph's linedraw.py (itself derived from
 * https://github.com/LingDong-/linedraw by Lingdong Huang), so that images can be turned into
 * plottable lines in the browser instead of on the ESP32.
 *
 *   vectorise(gray, {resolution: 1024, draw_contours: 2, repeat_contours: 1,
 *                    draw_hatch: 16, repeat_hatch: 1})
 *
 * `gray` is {width, height, data: Uint8Array} (greyscale, 0 = black). Edge detection follows the
 * OpenCV path of linedraw.py (3x3 Gaussian blur + Canny 100/200). Also included: SVG path
 * tracing (vector input) and Ramer-Douglas-Peucker simplification.
 */
(function (root) {
  "use strict";

  // ----------------------------------------------------------------- image helpers

  // PIL's "L" conversion: L = R * 299/1000 + G * 587/1000 + B * 114/1000
  function rgbaToGray(rgba, width, height) {
    const data = new Uint8Array(width * height);
    for (let i = 0, j = 0; i < data.length; i++, j += 4) {
      // composite transparent pixels onto white
      const a = rgba[j + 3] / 255;
      const r = rgba[j] * a + 255 * (1 - a);
      const g = rgba[j + 1] * a + 255 * (1 - a);
      const b = rgba[j + 2] * a + 255 * (1 - a);
      data[i] = (r * 19595 + g * 38470 + b * 7471 + 0x8000) >> 16;
    }
    return { width, height, data };
  }

  // PIL ImageOps.autocontrast(image, cutoff)
  function autocontrast(img, cutoff) {
    const h = new Array(256).fill(0);
    for (const v of img.data) h[v]++;
    const n = img.data.length;
    if (cutoff) {
      let cut = Math.floor((n * cutoff) / 100);
      for (let lo = 0; lo < 256; lo++) {
        if (cut > h[lo]) { cut -= h[lo]; h[lo] = 0; } else { h[lo] -= cut; cut = 0; }
        if (cut <= 0) break;
      }
      cut = Math.floor((n * cutoff) / 100);
      for (let hi = 255; hi >= 0; hi--) {
        if (cut > h[hi]) { cut -= h[hi]; h[hi] = 0; } else { h[hi] -= cut; cut = 0; }
        if (cut <= 0) break;
      }
    }
    let lo = 0, hi = 255;
    while (lo < 256 && !h[lo]) lo++;
    while (hi >= 0 && !h[hi]) hi--;
    const lut = new Uint8Array(256);
    if (hi <= lo) {
      for (let i = 0; i < 256; i++) lut[i] = i;
    } else {
      const scale = 255 / (hi - lo), offset = -lo * scale;
      for (let i = 0; i < 256; i++) {
        let v = Math.trunc(i * scale + offset);
        lut[i] = v < 0 ? 0 : v > 255 ? 255 : v;
      }
    }
    const out = new Uint8Array(n);
    for (let i = 0; i < n; i++) out[i] = lut[img.data[i]];
    return { width: img.width, height: img.height, data: out };
  }

  // Resize a greyscale image. In the browser this uses a canvas (smooth, high quality);
  // elsewhere (tests) a simple area-average. Can be overridden via Linedraw.resizeImpl.
  function resizeGray(img, w, h) {
    if (api.resizeImpl) return api.resizeImpl(img, w, h);
    if (typeof document !== "undefined") {
      const src = document.createElement("canvas");
      src.width = img.width; src.height = img.height;
      const sctx = src.getContext("2d");
      const id = sctx.createImageData(img.width, img.height);
      for (let i = 0, j = 0; i < img.data.length; i++, j += 4) {
        id.data[j] = id.data[j + 1] = id.data[j + 2] = img.data[i];
        id.data[j + 3] = 255;
      }
      sctx.putImageData(id, 0, 0);
      const dst = document.createElement("canvas");
      dst.width = w; dst.height = h;
      const dctx = dst.getContext("2d");
      dctx.imageSmoothingEnabled = true;
      dctx.imageSmoothingQuality = "high";
      dctx.drawImage(src, 0, 0, w, h);
      const out = dctx.getImageData(0, 0, w, h).data;
      const data = new Uint8Array(w * h);
      for (let i = 0, j = 0; i < data.length; i++, j += 4) data[i] = out[j];
      return { width: w, height: h, data };
    }
    const data = new Uint8Array(w * h);
    const sx = img.width / w, sy = img.height / h;
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const x0 = Math.floor(x * sx), x1 = Math.max(x0 + 1, Math.floor((x + 1) * sx));
        const y0 = Math.floor(y * sy), y1 = Math.max(y0 + 1, Math.floor((y + 1) * sy));
        let s = 0, c = 0;
        for (let yy = y0; yy < y1 && yy < img.height; yy++)
          for (let xx = x0; xx < x1 && xx < img.width; xx++) { s += img.data[yy * img.width + xx]; c++; }
        data[y * w + x] = Math.round(s / c);
      }
    }
    return { width: w, height: h, data };
  }

  // linedraw.resize_image
  function resizeImage(img, resolution, divider) {
    divider = divider || 1;
    const w = Math.trunc(resolution / divider);
    const h = Math.trunc(((resolution / divider) * img.height) / img.width);
    return resizeGray(img, w, h);
  }

  // ----------------------------------------------------------------- edge detection

  // cv2.GaussianBlur(im, (3, 3), 0): fixed kernel [1/4, 1/2, 1/4], BORDER_REFLECT_101
  function gaussianBlur3(img) {
    const { width: w, height: h, data } = img;
    const refl = (i, n) => (n === 1 ? 0 : i < 0 ? -i : i >= n ? 2 * n - 2 - i : i);
    const tmp = new Float32Array(w * h);
    for (let y = 0; y < h; y++)
      for (let x = 0; x < w; x++)
        tmp[y * w + x] =
          0.25 * data[y * w + refl(x - 1, w)] + 0.5 * data[y * w + x] + 0.25 * data[y * w + refl(x + 1, w)];
    const out = new Uint8Array(w * h);
    for (let y = 0; y < h; y++)
      for (let x = 0; x < w; x++) {
        const v = 0.25 * tmp[refl(y - 1, h) * w + x] + 0.5 * tmp[y * w + x] + 0.25 * tmp[refl(y + 1, h) * w + x];
        out[y * w + x] = Math.round(v);
      }
    return { width: w, height: h, data: out };
  }

  // cv2.Canny(im, low, high) with a 3x3 Sobel aperture and the L1 gradient norm
  function canny(img, low, high) {
    const { width: w, height: h, data } = img;
    const at = (x, y) => data[Math.min(h - 1, Math.max(0, y)) * w + Math.min(w - 1, Math.max(0, x))];
    const dx = new Int32Array(w * h), dy = new Int32Array(w * h), mag = new Int32Array(w * h);
    for (let y = 0; y < h; y++)
      for (let x = 0; x < w; x++) {
        const a = at(x - 1, y - 1), b = at(x, y - 1), c = at(x + 1, y - 1);
        const d = at(x - 1, y), f = at(x + 1, y);
        const g = at(x - 1, y + 1), hh = at(x, y + 1), k = at(x + 1, y + 1);
        const gx = c + 2 * f + k - a - 2 * d - g;
        const gy = g + 2 * hh + k - a - 2 * b - c;
        const i = y * w + x;
        dx[i] = gx; dy[i] = gy; mag[i] = Math.abs(gx) + Math.abs(gy);
      }
    // non-maximum suppression (same direction test as OpenCV)
    const TG22 = 13573; // tan(22.5°) * 2^15
    const state = new Uint8Array(w * h); // 0 none, 1 weak, 2 strong
    const stack = [];
    const M = (x, y) => (x < 0 || y < 0 || x >= w || y >= h ? 0 : mag[y * w + x]);
    for (let y = 0; y < h; y++)
      for (let x = 0; x < w; x++) {
        const i = y * w + x, m = mag[i];
        if (m <= low) continue;
        const xs = Math.abs(dx[i]), ys = Math.abs(dy[i]) << 15;
        const tg22x = xs * TG22, tg67x = tg22x + (xs << 16);
        let keep;
        if (ys < tg22x) keep = m > M(x - 1, y) && m >= M(x + 1, y);
        else if (ys > tg67x) keep = m > M(x, y - 1) && m >= M(x, y + 1);
        else {
          const s = (dx[i] ^ dy[i]) < 0 ? -1 : 1;
          keep = m > M(x - s, y - 1) && m > M(x + s, y + 1);
        }
        if (!keep) continue;
        if (m > high) { state[i] = 2; stack.push(i); } else state[i] = 1;
      }
    // hysteresis
    while (stack.length) {
      const i = stack.pop(), x = i % w, y = (i - x) / w;
      for (let oy = -1; oy <= 1; oy++)
        for (let ox = -1; ox <= 1; ox++) {
          const nx = x + ox, ny = y + oy;
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
          const j = ny * w + nx;
          if (state[j] === 1) { state[j] = 2; stack.push(j); }
        }
    }
    const out = new Uint8Array(w * h);
    for (let i = 0; i < out.length; i++) out[i] = state[i] === 2 ? 255 : 0;
    return { width: w, height: h, data: out };
  }

  // linedraw.find_edges
  function findEdges(img) {
    const edges = canny(gaussianBlur3(img), 100, 200);
    for (let i = 0; i < edges.data.length; i++) edges.data[i] = edges.data[i] > 128 ? 255 : 0;
    return edges;
  }

  // ----------------------------------------------------------------- contours

  function getdots(px, w, h) {
    const dots = [];
    for (let y = 0; y < h - 1; y++) {
      const row = [];
      for (let x = 1; x < w; x++) {
        if (px(x, y) === 255) {
          const last = row[row.length - 1];
          if (last && x - last[0] === last[1] + 1) last[1] += 1;
          else row.push([x, 0]);
        }
      }
      dots.push(row);
    }
    return dots;
  }

  function connectdots(dots) {
    const contours = [];
    for (let y = 0; y < dots.length; y++) {
      for (const [x] of dots[y]) {
        if (y === 0) { contours.push([[x, y]]); continue; }
        let closest = -1, cdist = 100;
        for (const [x0] of dots[y - 1]) {
          if (Math.abs(x0 - x) < cdist) { cdist = Math.abs(x0 - x); closest = x0; }
        }
        if (cdist > 3) { contours.push([[x, y]]); continue; }
        let found = false;
        for (let i = 0; i < contours.length; i++) {
          const c = contours[i], last = c[c.length - 1];
          if (last[0] === closest && last[1] === y - 1) { c.push([x, y]); found = true; break; }
        }
        if (!found) contours.push([[x, y]]);
      }
      // drop short contours that have ended (mirrors Python's remove-while-iterating behaviour,
      // which skips the element after each removal)
      for (let i = 0; i < contours.length; i++) {
        const c = contours[i];
        if (c[c.length - 1][1] < y - 1 && c.length < 4) contours.splice(i, 1);
      }
    }
    return contours;
  }

  const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

  function getcontours(img, drawContours) {
    const edges = findEdges(img);
    const { width: w, height: h, data } = edges;
    const contours1 = connectdots(getdots((x, y) => data[y * w + x], w, h));
    // the second pass works on the transposed image, to catch near-horizontal edges
    const contours2 = connectdots(getdots((x, y) => data[x * w + y], h, w));
    for (const c of contours2) for (const p of c) { const t = p[0]; p[0] = p[1]; p[1] = t; }
    let contours = contours1.concat(contours2);

    for (let i = 0; i < contours.length; i++) {
      for (let j = 0; j < contours.length; j++) {
        if (contours[i].length > 0 && contours[j].length > 0) {
          if (dist(contours[j][0], contours[i][contours[i].length - 1]) < 8) {
            contours[i] = contours[i].concat(contours[j]);
            contours[j] = [];
          }
        }
      }
    }

    contours = contours.map((c) => c.filter((_, j) => j % 8 === 0)).filter((c) => c.length > 1);
    return contours.map((c) => c.map((v) => [v[0] * drawContours, v[1] * drawContours]));
  }

  // ----------------------------------------------------------------- hatching

  const E = [1, 0], S = [0, 1], SE = [1, 1], NE = [1, -1];

  function getLines(img, scan, direction, spacing, level) {
    const { width, height, data } = img;
    let iStart = 0, jStart = 0, iRange;
    const lines = [];
    if (scan === "y") iRange = height;
    else {
      iRange = width;
      if (direction === SE) iStart = spacing;
      else if (direction === NE) {
        // (as in linedraw.py: `height - 1 % line_spacing` == height - 1)
        iStart = spacing - (height - 1);
        jStart = height - 1;
      }
    }
    for (let i = iStart; i < iRange; i += spacing) {
      let start = null, end = null, x, y;
      if (scan === "y") { x = jStart; y = i; } else { x = i; y = jStart; }
      while (x >= 0 && x < width && y >= 0 && y < height) {
        const v = data[y * width + x];
        if (!start) { if (v < level) start = [x, y]; }
        else if (v >= level) { lines.push([start, [x, y]]); start = null; }
        end = [x, y];
        x += direction[0]; y += direction[1];
      }
      if (start) lines.push([start, end]);
    }
    return lines;
  }

  function hatch(img, spacing) {
    return [].concat(
      getLines(img, "y", E, spacing, 160),
      getLines(img, "x", S, spacing, 80),
      getLines(img, "y", SE, spacing, 40),
      getLines(img, "x", SE, spacing, 40),
      getLines(img, "y", NE, spacing, 20),
      getLines(img, "x", NE, spacing, 20)
    );
  }

  // ----------------------------------------------------------------- pen-path optimisation

  // greedy nearest-neighbour ordering, reversing lines when their far end is closer
  function sortlines(lines) {
    if (!lines.length) return [];
    const clines = lines.slice();
    const slines = [clines.shift()];
    while (clines.length) {
      const end = slines[slines.length - 1][slines[slines.length - 1].length - 1];
      let best = -1, s = Infinity, rev = false;
      for (let k = 0; k < clines.length; k++) {
        const l = clines[k];
        const a = l[0], b = l[l.length - 1];
        const d = (a[0] - end[0]) ** 2 + (a[1] - end[1]) ** 2;
        const dr = (b[0] - end[0]) ** 2 + (b[1] - end[1]) ** 2;
        if (d < s) { best = k; s = d; rev = false; }
        if (dr < s) { best = k; s = dr; rev = true; }
      }
      let x = clines.splice(best, 1)[0].slice();
      if (rev) x.reverse();
      slines.push(x);
    }
    return slines;
  }

  function joinLines(lines, closeness) {
    closeness = closeness || 128;
    const out = [];
    let prev = null;
    for (const line of lines) {
      if (!prev) { prev = line.slice(); out.push(prev); continue; }
      const dx = prev[prev.length - 1][0] - line[0][0];
      const dy = prev[prev.length - 1][1] - line[0][1];
      if (dx * dx + dy * dy <= closeness) prev.push(...line);
      else { prev = line.slice(); out.push(prev); }
    }
    return out;
  }

  // ----------------------------------------------------------------- main entry point

  function vectorise(gray, opts) {
    opts = Object.assign(
      { resolution: 1024, draw_contours: 2, repeat_contours: 1, draw_hatch: 16, repeat_hatch: 1 },
      opts || {}
    );
    const report = opts.onProgress || (() => {});
    const image = autocontrast(gray, 5);
    let lines = [];
    if (opts.draw_contours && opts.repeat_contours) {
      report("Generating contours…");
      let contours = getcontours(resizeImage(image, opts.resolution, opts.draw_contours), opts.draw_contours);
      report("Optimising contour sequence…");
      contours = joinLines(sortlines(contours));
      for (let r = 0; r < opts.repeat_contours; r++) lines = lines.concat(contours);
    }
    if (opts.draw_hatch && opts.repeat_hatch) {
      report("Hatching…");
      let hatches = hatch(resizeImage(image, opts.resolution), opts.draw_hatch);
      report("Optimising hatch sequence…");
      hatches = joinLines(sortlines(hatches));
      for (let r = 0; r < opts.repeat_hatch; r++) lines = lines.concat(hatches);
    }
    return lines;
  }

  // ----------------------------------------------------------------- vector (SVG) input

  // Ramer-Douglas-Peucker
  function simplify(points, eps) {
    if (points.length < 3 || !eps) return points;
    const keep = new Uint8Array(points.length);
    keep[0] = keep[points.length - 1] = 1;
    const stack = [[0, points.length - 1]];
    while (stack.length) {
      const [a, b] = stack.pop();
      const [ax, ay] = points[a], [bx, by] = points[b];
      const len = Math.hypot(bx - ax, by - ay);
      let maxD = -1, idx = -1;
      for (let i = a + 1; i < b; i++) {
        const [px, py] = points[i];
        const d = len
          ? Math.abs((bx - ax) * (ay - py) - (ax - px) * (by - ay)) / len
          : Math.hypot(px - ax, py - ay);
        if (d > maxD) { maxD = d; idx = i; }
      }
      if (maxD > eps) { keep[idx] = 1; stack.push([a, idx], [idx, b]); }
    }
    return points.filter((_, i) => keep[i]);
  }

  // Trace the shapes of an SVG that has been attached to the document.
  // Returns lines in the SVG's own (y-down) co-ordinates.
  function traceSvgElement(svg, opts) {
    opts = Object.assign({ samples: 1500, tolerance: 0.25 }, opts || {});
    const skip = "defs, clipPath, mask, symbol, marker, pattern, title, desc, metadata";
    const shapes = Array.from(svg.querySelectorAll("path, line, polyline, polygon, rect, circle, ellipse"))
      .filter((el) => !el.closest(skip));
    const pt = svg.createSVGPoint();
    const raw = [];
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;

    const toRoot = (el) => {
      const m = el.getCTM();
      const r = svg.getCTM();
      return r && m ? r.inverse().multiply(m) : m;
    };

    let totalLength = 0;
    const lengths = shapes.map((el) => {
      let l = 0;
      try { l = el.getTotalLength(); } catch (e) { l = 0; }
      totalLength += l;
      return l;
    });
    if (!totalLength) return [];
    const step = totalLength / Math.max(200, opts.samples * shapes.length) * 4;

    shapes.forEach((el, k) => {
      const m = toRoot(el);
      const map = (x, y) => {
        pt.x = x; pt.y = y;
        const q = m ? pt.matrixTransform(m) : pt;
        return [q.x, q.y];
      };
      const tag = el.tagName.toLowerCase();
      let polys = [];
      if (tag === "polyline" || tag === "polygon") {
        const pts = Array.from(el.points, (p) => map(p.x, p.y));
        if (tag === "polygon" && pts.length) pts.push(pts[0]);
        polys.push(pts);
      } else if (tag === "line") {
        polys.push([map(el.x1.baseVal.value, el.y1.baseVal.value), map(el.x2.baseVal.value, el.y2.baseVal.value)]);
      } else {
        const len = lengths[k];
        if (!len) return;
        const n = Math.max(8, Math.ceil(len / step));
        const ds = len / n;
        let cur = [], prev = null;
        for (let i = 0; i <= n; i++) {
          const p = el.getPointAtLength(Math.min(len, i * ds));
          // a jump bigger than the sampling step means a "move to" inside the path
          if (prev && Math.hypot(p.x - prev.x, p.y - prev.y) > ds * 1.5 + 1e-6) {
            if (cur.length > 1) polys.push(cur);
            cur = [];
          }
          cur.push(map(p.x, p.y));
          prev = p;
        }
        if (cur.length > 1) polys.push(cur);
      }
      for (const poly of polys) {
        if (poly.length < 2) continue;
        for (const [x, y] of poly) {
          if (x < minX) minX = x; if (x > maxX) maxX = x;
          if (y < minY) minY = y; if (y > maxY) maxY = y;
        }
        raw.push(poly);
      }
    });

    const diag = Math.hypot(maxX - minX, maxY - minY) || 1;
    const eps = (diag / 1000) * opts.tolerance;
    return sortlines(raw.map((p) => simplify(p, eps)).filter((p) => p.length > 1));
  }

  // ----------------------------------------------------------------- serialisation

  // .lines format: one polyline per text line, "x,y x,y ..."; long lines are split into chunks
  // that overlap by one point (so the pen stays down) to keep ESP32 memory use small.
  function toLinesText(lines, maxPoints) {
    maxPoints = maxPoints || 200;
    const fmt = (v) => String(Math.round(v * 100) / 100);
    const out = [];
    for (const line of lines) {
      if (!line.length) continue;
      for (let start = 0; ; start += maxPoints - 1) {
        const chunk = line.slice(start, start + maxPoints);
        out.push(chunk.map((p) => fmt(p[0]) + "," + fmt(p[1])).join(" "));
        if (start + maxPoints >= line.length) break;
      }
    }
    return out.join("\n") + "\n";
  }

  function parseLinesText(text) {
    const lines = [];
    for (const row of text.split("\n")) {
      const t = row.trim();
      if (!t || t[0] === "#") continue;
      const pts = t.split(/\s+/).map((tok) => tok.split(",").map(Number));
      // re-join chunks that continue the previous line
      const prev = lines[lines.length - 1];
      if (prev) {
        const a = prev[prev.length - 1];
        if (a[0] === pts[0][0] && a[1] === pts[0][1]) { prev.push(...pts.slice(1)); continue; }
      }
      lines.push(pts);
    }
    return lines;
  }

  function stats(lines) {
    let points = 0, drawn = 0, travel = 0, prev = null;
    for (const l of lines) {
      points += l.length;
      if (prev) travel += dist(prev, l[0]);
      for (let i = 1; i < l.length; i++) drawn += dist(l[i - 1], l[i]);
      prev = l[l.length - 1];
    }
    return { lines: lines.length, points, segments: points - lines.length, drawn, travel };
  }

  const api = {
    rgbaToGray, autocontrast, resizeImage, resizeGray, gaussianBlur3, canny, findEdges,
    getdots, connectdots, getcontours, hatch, getLines, sortlines, joinLines, vectorise,
    simplify, traceSvgElement, toLinesText, parseLinesText, stats, resizeImpl: null,
  };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  root.Linedraw = api;
})(typeof window !== "undefined" ? window : globalThis);
