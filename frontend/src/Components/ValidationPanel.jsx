import React, { useState } from 'react';
import Plot from 'react-plotly.js';
import { propagateTrajectory, validatePropagators, createState } from '../api';
import { ISS_ALTITUDE, ISS_PERIOD, darkLayout, dark3DLayout, PLOTLY_CONFIG, COLORS } from '../constaints';

const PRESETS = {
    football: { label: 'Football (R-bar offset)', state: { x: 1000, y: 0, z: 0, vx: 0, vy: 0, vz: 0 } },
    teardrop: { label: 'Teardrop (V-bar offset)', state: { x: 0, y: -1000, z: 0, vx: 0, vy: 0, vz: 0 } },
    vbarHop: { label: 'V-bar Hop', state: { x: 0, y: -500, z: 0, vx: 0.5, vy: 0, vz: 0 } },
    crossTrack: { label: 'Cross-track', state: { x: 0, y: 0, z: 500, vx: 0, vy: 0, vz: 0 } },
};

function ValidationPanel() {
    const [preset, setPreset] = usedState('football');
    const [x0, setX0] = useState({ ...PRESETS.fottball.state });
    const [durationOrbits, setDurationOrbits] = useState(2.0);
    const [numPoints, setNumPoints] = useState(500);
    const [numSteps, setNumSteps] = useState(1000);

    const [loading, setLoading] = useState(false);
    const [traj, setTraj] = useState(null);
    const [valResult, setValResult] = useState(null);
    const [error, setError] = useState(null);

    const handlePreset = (key) => {
        setPreset(key);
        setX0({ ...PRESETS[key].state });
    };

    const updateX0 = (field, value) => {
        setPreset('');
        setX0(prev => ({ ...prev, [field]: parseFloat(value) || 0}));
    };

    const run = async () => {
        setLoading(true);
        setError(null);
        try {
            const duration = durationOrbits * ISS_PERIOD;
            const state = createState(x0.x, x0.y, x0.z, x0.vx, x0.vy, x0.vz);

            const [propResult, valData] = await Promise.all([
                propagateTrajectory({
                    initialState: state,
                    duration,
                    altitude: ISS_ALTITUDE,
                    numPoints,
                }),
                validatePropagators({
                    initialState: state,
                    duration,
                    altitude: ISS_ALTITUDE,
                    numSteps,
                }),
            ]);

            setTraj(propResult.trajectory);
            setValResult(valData);
        } catch (e) {
            setError(e.message);
        } finally {
            setLoading(false);
        }
    };

    // --- trajectory traces (3D) ---
    const traces = [];
    
    if (traj) {
        traces.push({
            x: traj.map(p => p.y),
            y: traj.map(p => p.x),
            z: traj.map(p => p.z),
            mode: 'lines', name: 'Analytical (STM)',
            type: 'scatter3d',
            line: { color: COLORS.cyan, width: 3 },
        });
        // Start marker
        traces.push({
            x: [traj[0].y],
            y: [traj[0].x],
            z: [traj[0].z],
            mode: 'markers',
            name: 'Start',
            marker: { color: COLORS.green, size: 5, symbol: 'diamond' },
        });
        // Target
        traces.push({
            x: [0], y: [0], z: [0],
            mode: 'markers',
            name: 'Target',
            type: 'scatter3d',
            marker: { color: COLORS.amber, size: 5, symbol: 'cross' },
        });
    }

    return (
        <div className="panel">
            <div className="panel-header">
                <h2>Propagator Validation</h2>
                <p>Analytical (STM) vs. Numerical (RK4)</p>
            </div>

            <div className="input-selection">
                <h4>Preset</h4>
                <div className="preset-row">
                    {Object.entries(PRESETS).map(([key, p]) => (
                        <button key={key}
                            className={`preset-btn ${preset === key ? 'active': ''}`}
                            onClick={() => handlePreset(key)}>
                            {p.label}
                        </button>
                    ))}
                </div>

                <h4>Initial State (LVLH)</h4>
                <div className="input-row">
                    {['x', 'y', 'z'].map(k => (
                        <label key={k}>{k}:
                            <input type="number" value={x0[k]}
                                onChange={e => updateX0(k, e.target.value)}
                            />
                            m
                        </label>
                    ))}
                </div>
                <div className="input-row">
                    {['vx', 'vy', 'vz'].map(k => (
                        <label key={k}>{k}:
                            <input type="number" value={x0[k]} step="0.01"
                                onChange={e => updateX0(k, e.target.value)}
                            />
                            m/s
                        </label>
                    ))}
                </div>

                <h4>Simulation</h4>
                <div className="input-row">
                    <label>
                        Duration:
                        <input type="number" value={durationOrbits} step="0.5" min="0.5"
                            onChange={e => setDurationOrbits(parseFloat(e.target.value) || 2)}
                        />
                        orbits
                    </label>
                    <label>
                        Plot points:
                        <input type="number" value={numPoints} step="10" min="50"
                            onChange={e => setNumPoints(parseInt(e.target.value) || 500)}
                        />
                    </label>
                    <label>
                        Validation steps:
                        <input type="number" value={numSteps} step="100" min="100"
                            onChange={e => setNumSteps(parseInt(e.target.value) || 100)}
                        />
                    </label>
                </div>
            </div>

            <button className="run-btn" onClick={run} disabled={loading}>
                {loading ? 'Validating...' : 'Run Propagation + Validation'}
            </button>

            {error && <div className="error-msg">{error}</div>}

            {valResult && (
                <>
                    <div className="metric-row">
                        <div className={`metric ${valResult.passed ? 'highlight-ok' : 'highlight-err'}`}>
                            <span className="metric-label">Cross-Reference</span>
                            <span className="metric-value">{valResult.passed ? 'PASSED' : 'FAILED'}</span>
                        </div>
                        <div className="metric">
                            <span className="metric-label">Max Position Error</span>
                            <span className="metric-value">{valResult.maxPositionError.toExponential(2)} m</span>
                        </div>
                        <div className="metric">
                            <span className="metric-label">Max Velocity Error</span>
                            <span className="metric-value">{valResult.maxVelocityError.toExponential(2)} m/s</span>
                        </div>
                        <div className="metric">
                            <span className="metric-label">Max Relative Position Error</span>
                            <span className="metric-value">{valResult.maxRelativePositionError.toExponential(2)}</span>
                        </div>
                    </div>
                </>
            )}

            {traj && (
                <div className="plot-container">
                    <Plot
                        data={traces}
                        layout={dark3DLayout({
                            title: { text: 'CW Relative Trajectory', font: { color: '#c8c8d8' } },
                        })}
                        config={PLOTLY_CONFIG}
                        style={{ width: '100%', height: '500px' }}
                    />
                </div>
            )}
        </div>
    );
}

export default ValidationPanel;