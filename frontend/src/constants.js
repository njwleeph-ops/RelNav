/**
 * @file constants.js
 * @brief Shared constants, Plotly theme, and ISS orbital defaults
 */

// Plot colors — muted, professional
export const COLORS = {
    trajectory: '#1a56db',
    success: '#16a34a',
    failure: '#dc2626',
    target: '#000000',
    cone: 'rgba(100, 120, 180, 0.12)',
    coneLine: 'rgba(100, 120, 180, 0.4)',
    amber: '#d97706',
    grid: '#cccccc',
    successSphere: 'rgba(22, 163, 74, 0.3)',
};

// Plotly config — no watermark, minimal toolbar
export const PLOTLY_CONFIG = {
    displaylogo: false,
    modeBarButtonsToRemove: ['sendDataToCloud'],
    responsive: true,
};

// Base layout for 2D plots
export function layout2D(overrides = {}) {
    return {
        font: { family: 'Times New Roman, serif', size: 12, color: '#222' },
        paper_bgcolor: '#ffffff',
        plot_bgcolor: '#ffffff',
        margin: { t: 40, r: 20, b: 50, l: 60 },
        xaxis: {
            gridcolor: COLORS.grid,
            zerolinecolor: '#999',
            zerolinewidth: 1,
            ...(overrides.xaxis || {}),
        },
        yaxis: {
            gridcolor: COLORS.grid,
            zerolinecolor: '#999',
            zerolinewidth: 1,
            ...(overrides.yaxis || {}),
        },
        ...overrides,
    };
}

// Base layout for 3D plots — equal axes, target at origin
export function layout3D(overrides = {}) {
    return {
        font: { family: 'Times New Roman, serif', size: 11, color: '#222' },
        paper_bgcolor: '#ffffff',
        margin: { t: 30, r: 10, b: 10, l: 10 },
        scene: {
            bgcolor: '#ffffff',
            xaxis: {
                title: 'R-bar [m]',
                gridcolor: COLORS.grid,
                zerolinecolor: '#999',
                ...(overrides.scene?.xaxis || {}),
            },
            yaxis: {
                title: 'V-bar [m]',
                gridcolor: COLORS.grid,
                zerolinecolor: '#999',
                ...(overrides.scene?.yaxis || {}),
            },
            zaxis: {
                title: 'H-bar [m]',
                gridcolor: COLORS.grid,
                zerolinecolor: '#999',
                ...(overrides.scene?.zaxis || {}),
            },
            aspectmode: 'data',
            ...(overrides.scene || {}),
        },
        ...overrides,
    };
}

// Determine which state component to lock negative based on approach axis
export function getAxisConstraint(approachAxis) {
    if (!approachAxis) return null;
    const ax = approachAxis;
    if (Math.abs(ax[1]) > 0.9) return 'y';   // V-bar
    if (Math.abs(ax[0]) > 0.9) return 'x';   // R-bar
    if (Math.abs(ax[2]) > 0.9) return 'z';   // H-bar
    return null;
}

// Get axis label from config approach_axis vector
export function getAxisLabel(approachAxis) {
    if (!approachAxis) return 'Unknown';
    if (Math.abs(approachAxis[1]) > 0.9) return 'V-bar';
    if (Math.abs(approachAxis[0]) > 0.9) return 'R-bar';
    if (Math.abs(approachAxis[2]) > 0.9) return 'H-bar';
    return 'Custom';
}

// Default initial states per approach axis
export function getDefaultState(approachAxis) {
    if (!approachAxis) return { x: 0, y: -500, z: 0, vx: 0, vy: 0.5, vz: 0 };
    if (Math.abs(approachAxis[1]) > 0.9) return { x: 0, y: -500, z: 0, vx: 0, vy: 0.5, vz: 0 };
    if (Math.abs(approachAxis[0]) > 0.9) return { x: -500, y: 0, z: 0, vx: 0.5, vy: 0, vz: 0 };
    if (Math.abs(approachAxis[2]) > 0.9) return { x: 0, y: 0, z: -500, vx: 0, vy: 0, vz: 0.5 };
    return { x: 0, y: -1000, z: 0, vx: 0, vy: 0.5, vz: 0 };
}

// Compute symmetric axis ranges for 3D plots, ensuring origin is visible
export function computeSymmetricRanges(points) {
    if (!points || points.length === 0) return {};

    let maxAbs = 0;
    for (const pt of points) {
        maxAbs = Math.max(maxAbs, Math.abs(pt.x || 0), Math.abs(pt.y || 0), Math.abs(pt.z || 0));
    }

    const pad = maxAbs * 0.1 + 1;
    const r = maxAbs + pad;

    return {
        scene: {
            xaxis: { range: [-r, r] },
            yaxis: { range: [-r, r] },
            zaxis: { range: [-r, r] },
        },
    };
}