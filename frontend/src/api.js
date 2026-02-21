/**
 * API Hanlders for RelNav backend
 */

const API_URL = 'http://localhost:8080/api';


/**
 * GET /api/health
 */
export async function getHealth() {
    const res = await fetch(`${API_URL}/health`);

    if (!res.ok) {
        const err = await res.json().catch(() => ({ error: res.statusText }));
        throw new Error(err.error || `Health check failed (${res.status})`);
    }

    return res.json();
}


/**
 * GET /api/orbit
 */
export async function getOrbit(altitude = 420e3) {
    const res = await fetch(`${API_URL}/orbit?altitude=${altitude}`);

    if (!res.ok) { 
        const err = await res.json().catch(() => ({ error: res.statusText }));
        throw new Error(err.error || `Failed to fetch orbit info (${res.status})`);
    }

    return res.json();
}

/**
 * POST /api/approach-guidance
 */
export async function runApproachGuidance(params) {
    const res = await fetch(`${API_URL}/approach-guidance`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(params)
    });

    if (!res.ok) {
        const err = await res.json().catch(() => ({ error: res.statusText }));
        throw new Error(err.error || `Approach guidance failed (${res.status})`);
    }

    return res.json();
}

/**
 * POST /api/monte-carlo
 */
export async function runMonteCarlo(params) {
    const res = await fetch(`${API_URL}/monte-carlo`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(params)
    });

    if (!res.ok) {
        const error = await res.json().catch(() => ({ error: res.statusText }));
        throw new Error(err.error || `Monte Carlo analysis failed (${res.status})`);
    }

    return res.json();
}

/**
 * POST /api/envelope
 */
export async function startEnvelope(params) {
    const res = await fetch(`${API_URL}/envelope`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(params)
    });

    if (!res.ok) {
        const err = await res.json().catch(() => ({ error: res.statusText }));
        throw new Error(err.error || `/envelope failed (${res.status})`);
    }

    return res.json();
}

/**
 * GET /api/envelope/:id
 */
export async function getEnvelopeStatus(jobId) {
    const res = await fetch(`${API_URL}/envelope/${jobId}`);

    if (!res.ok) {
        const err = await res.json().catch(() => ({ error: res.statusText }));
        throw new Error(err.error || `/envelope/${jobId} failed (${res.status})`);
    }

    return res.json();
}

/**
 * DELETE /api/envelope/:id
 */
export async function deleteEnvelope(jobId) {
    const res = await fetch(`${API_URL}/envelope/${jobId}`, { method: 'DELETE' });

    if (!res.ok) {
        const err = await res.json().catch(() => ({ error: res.statusText }));
        throw new Error(err.error || `DELETE envelope/${jobId} failed`);
    }

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

    if (!res.ok) {
        const err = await res.json().catch(() => ({ error: res.statusText }));
        throw new Error(err.error || `Propagation failed (${res.status})`);
    }
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