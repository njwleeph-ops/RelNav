/**
 * API Hanlders for RelNav backend
 */

const API_URL = 'http://localhost:8080/api';


/**
 * GET /api/health
 */
export async function getHealth() {
    const res = await fetch(`${API_URL}/health`);

    if (!res.ok) throw new Error('Health check failed');

    return res.json();
}


/**
 * GET /api/orbit
 */
export async function getOrbit(altitude = 420e3) {
    const res = await fetch(`${API_URL}/orbit?altitude=${altitude}`);

    if (!res.ok) throw new Error('Failed to fetch orbit info');

    return res.json();
}


/**
 * POST /api/propagate
 */
export async function propagateTrajectory(params) {
    const res = await fetch(`${API_URL}/propagate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(params)
    });

    if (!res.ok) throw new Error('Propagation failed');
    
    return res.json();
}


/**
 * POST /api/validate
 */
export async function validatePropagators(params) {
    const res = await fetch(`${API_URL}/validate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(params)
    });

    if (!res.ok) throw new Error('Validation failed');

    return res.json();
}


/**
 * POST /api/two-impulse
 */
export async function simulateTwoImpulse(params) {
    const res = await fetch(`${API_URL}/two-impulse`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(params)
    });

    if (!res.ok) throw new Error('Two-impulse targeting failed');

    return res.json();
}


/**
 * POST /api/targeted-monte-carlo
 */
export async function runTargetedMonteCarlo(params) {
    const res = await fetch(`${API_URL}/targeted-monte-carlo`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(params)
    });

    if (!res.ok) throw new Error('Targeted Monte Carlo analysis failed');

    return res.json();
}

/**
 * POST /api/lqr-approach
 */
export async function simulateLQRApproach(params) {
    const res = await fetch(`${API_URL}/lqr-approach`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(params)
    });

    if (!res.ok) throw new Error('LQR simulation failed');

    return res.json();
}


/**
 * POST /api/glideslope-check
 */
export async function checkGlideslope(params) {
    const res = await fetch(`${API_URL}/glideslope-check`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(params)
    });

    if (!res.ok) throw new Error('Glideslope check failed');

    return res.json();
}


/**
 * POST /api/dv-sweep
 */
export async function getDVSweep(params) {
    const res = await fetch(`${API_URL}/dv-sweep`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(params)
    });

    if (!res.ok) throw new Error("DV Sweep failed");

    return res.json();
}


/**
 * Helper: create state object
 */
export function createState(x, y, z, vx = 0, vy = 0, vz = 0) {
    return { x, y, z, vx, vy, vz };
}


/**
 * Helper: create position object
 */
export function createPosition(x, y, z) {
    return { x, y, z };
}