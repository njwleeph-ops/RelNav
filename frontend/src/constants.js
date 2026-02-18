/**
 * @file constants.js
 * @brief Shared constants, Plotly theme, and ISS orbital defaults
 */

// --------------------------------------------------------------
// ISS Defaults
// --------------------------------------------------------------

export const ISS_ALTITUDE = 420e3;
export const ISS_PERIOD = 5569.4;

// --------------------------------------------------------------
// Plotly dark theme
// --------------------------------------------------------------

const GRID = '#252540';
const ZERO = '#3a3a5c';
const TEXT = '#c8c8d8';
const SUBTLE = '#8888a0';

export function darkLayout(overrides = {}) {
    const base = {
        paper_bgcolor: '#0e0e1a',
        plot_bgcolor: '#0e0e1a',
        font: { color: TEXT, family: 'JetBrains Mono, monospace', size: 11 },
        margin: { t: 40, b: 50, l: 60, r: 20 },
        showlegend: true,
        legend: { font: { color: SUBTLE, size: 10 },  bgcolor: 'rgba(0,0,0,0)' },
    };

    return {
        ...base,
        ...overrides,
        xaxis: { color: SUBTLE, gridcolor: GRID, zerolinecolor: ZERO, ...(overrides.xaxis || {}) },
        yaxis: { color: SUBTLE, gridcolor: GRID, zerolinecolor: ZERO, ...(overrides.yaxis || {}) },
    };
}

export function dark3DLayout(overrides = {}) {
    const axisBase = {
        color: SUBTLE,
        gridcolor: GRID,
        zerolinecolor: ZERO,
        zerolinewidth: 2,
        backgroundcolor: '#0a0a16',
        showbackground: true,
        showspikes: false,
    };

    return {
        paper_bgcolor: '#0e0e1a',
        font: { color: TEXT, family: 'JetBrains Mono, monospace', size: 11 },
        margin: { t: 40, b: 10, l: 10, r: 10 },
        showlegend: true,
        legend: { font: { color: SUBTLE, size: 10 }, bgcolor: 'rgba(0,0,0,0)' },
        scene: {
            xaxis: { ...axisBase, title: 'V-bar [m]', ...(overrides.xaxis || {}) },
            yaxis: { ...axisBase, title: 'R-bar [m]', ...(overrides.yaxis || {}) },
            zaxis: { ...axisBase, title: 'H-bar [m]', ...(overrides.zaxis || {}) },
            bgcolor: '#0e0e1a',
            camera: overrides.camera || { eye: { x: 1.5, y: 1.5, z: 1.0 } },
            aspectmode: 'cube',
            ...(overrides.scene || {}),
        },
        ...overrides,
    };
}

export function computeSymmetricRanges(coords, padding = 0.15) {
    const { x, y, z } = coords;
    const maxAbs = Math.max(
        ...x.map(Math.abs),
        ...y.map(Math.abs),
        ...z.map(Math.abs),
        1
    );
    const bound = maxAbs * (1 + padding);
    const range = [-bound, bound];

    return {
        xaxis: { range },
        yaxis: { range },
        zaxis: { range },
    };
}

export const PLOTLY_CONFIG = { responsive: true, displayModeBar: false };

// --------------------------------------------------------------
// Color Palette
// --------------------------------------------------------------

export const COLORS = {
    cyan: '#00d4ff',
    magenta: '#ff3d8b',
    green: '#00ff88',
    amber: '#ffaa00',
    red: '#ff4444',
    white: '#e0e0f0',
    dim: '#555580',
};