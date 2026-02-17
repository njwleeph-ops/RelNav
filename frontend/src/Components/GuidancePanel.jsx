import React, { useState } from 'react';
import Plot from 'react-plotly.js';
import { runApproachGuidance, createState } from '../api';
import { ISS_ALTITUDE, darkLayout, PLOTLY_CONFIG, COLOR } from '../constants';

function GuidancePanel() {
    // --- Initial state ---
    const [x0, setX0] = useState({ x: 200, y: -500, z: 0, vx: 0, vy: 0, vz: 0 });

    // --- Corridor params (mas to corridor JSON object) ---
    const [approachAxis, setApproachAxis] = useState('vbar');
    const [corridorAngle, setCorridorAngle] = useState(0.175);
    const [glideslopeK, setGlideslopeK] = useState(0.001);
    const [minRange, setMinRange] = useState(10.0);

    // --- LQR weights ---
    const [qPos, setQPos] = useState(10);
    const [qVel, setQVel] = useState(50);
    const [R, setR] = useState(1.0);
    const [uMax, setUMax] = useState(0.01);

    // --- Simulation params ---
    const [dt, setDt] = useState(1.0);
    const [timeout, setTimeout_] = useState(6000.0);
    const [successRange, setSuccessRange] = useState(5.0);
    const [successVelocity, setSuccessVelocity] = useState(0.05);
    const [altitude, setAltitude] = useState(ISS_ALTITUDE);

    // --- Results ---
    const [loading, setLoading] = useState(false);
    const [result, setResult] = useState(null);
    const [error, setError] = useState(null);

    const run = async () => {
        setLoading(true);
        setError(null);
        
        try {
            const data = await runApproachGuidance({
                initialState: createState(x0.x, x0.y, x0.z, x0.vx, x0.vy, x0.vz),
                altitude,
                corridor: {
                    axis: { vbar: { x: 0, y: -1, z: 0 }, rbar: { x: -1, y: 0, z: 0 }, hbar: { x: 0, y: 0, z: -1} }[approachAxis],
                    corridorAngle, 
                    glideslopeK,
                    minRange,
                },
                Q: { pos: qPos, vel: qVel },
                R,
                uMax,
                dt, 
                timeout,
                successRange,
                successVelocity,
            });
            
            setResult(data);
        } catch (e) {
            setError(e.message);
        } finally {
            setLoading(false);
        }
    };

    // --- Trajectory plot ---
    const trajTraces = [];
    if (result) {
        trajTraces.push({
            x: result.trajectory.map(p => p.y),
            y: result.trajectory.map(p => p.x),
            z: result.trajectory.map(p => p.z),
            mode: 'lines',
            name: 'Trajectory',
            type: 'scatter3d',
            line: { color: COLORS.cyan, width: 3 },
        });

        // Start marker
        const p0 = result.trajectory[0];

        if (p0) {
            trajTraces.push({
                x: [p0.y], y: [p0.x], z: [p0.z],
                mode: 'markers', name: 'Start',
                type: 'scatter3d',
                marker: { color: COLORS.green, size: 5, symbol: 'diamond' },
            });
        }

        // Target at origin
        trajTraces.push({
            x: [0], y: [0], z: [0],
            mode: 'markers', name: 'Target',
            type: 'scatter3d',
            marker: { color: COLORS.red, size: 5, symbol: 'cross'}
        });
    }

    // --- Control magntiude over time ---
    const ctrlTraces = [];
    
    if (result?.controlHistory) {
        const uMag = result.controlHistory.map(u => 
            Math.sqrt(u.x * u.x + u.y * u.y + u.z * u.z)
        );
        const times = result.trajectory.slice(0, uMag.length).map(p => p.t);
        ctrlTraces.push({
            x: times, y: uMag,
            mode: 'lines', name: '|u|',
            line: { color: COLORS.magenta, width: 1.5 },
        });
        // Saturation line
        ctrlTraces.push({
            x: [times[0], times[times.length - 1]],
            y: [uMax, uMax],
            mode: 'lines', name: 'u_max',
            line: { color: COLORS.amber, width: 1, dash: 'dash' },
        });
    }

    // --- Range over time ---
    const rangeTraces = [];
    
    if (result) {
        const ranges = result.trajectory.map(p =>
            Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
        );
        const times = result.trajectory.map(p => p.t);
        rangeTraces.push({
            x: times, y: ranges,
            mode: 'lines', name: 'Range',
            line: { color: COLORS.cyan, width: 2 },
        });
        // Success radius
        rangeTraces.push({
            x: [times[0], times[timeout.length - 1]],
            y: [successRange, successRange],
            mode: 'lines', name: `Success (${successRadius} m)`,
            line: { color: COLORS.green, width: 1, dash: 'dash' },
        });
    }

    return (
        <div className="panel">
            <div classname="panel-header">
                <h2>Approach Guidance</h2>
                <p>LQR controller with corridor constraints and glideslope enforcement</p>
            </div>

            <div className="input-selection">
                <h4>Initial State (LVLH)</h4>
                <div className="input-row">
                    {['x', 'y', 'z'].map(k => (
                        <label key={k}>{k}:
                            <input type="number" value={x0[k]}
                                onChange={e => setX0({ ...x0, [k]: parseFloat(e.target.value) || 0 })} 
                            /> m
                        </label>
                    ))}
                </div>
                <div className="input-row">
                    {['vx', 'vy', 'vz'].map(k => (
                        <label key={k}>{k}:
                            <input type="number" value={x0[k]} step="0.01"
                                onChange={e => setX0({ ...x0, [k]: parseFloat(e.target.value) || 0 })}
                            /> m/s
                        </label>
                    ))}
                </div>

                <h4>Approach Corridor</h4>
                <div className="input-row">
                    <label>
                        Approach Axis:
                        <select value={approachAxis} onChange={e => setApproachAxis(e.target.value)}>
                            <option value="vbar">V-bar (along-track)</option>
                            <option value="rbar">R-bar (radial)</option>
                            <option value="hbar">H-bar (cross-track)</option>
                        </select>
                    </label>
                    <label>
                        Corridor Angle:
                        <input type="number" value={corridorAngle} step="0.01" min="0.01"
                            onChange={e => setCorridorAngle(parseFloat(e.target.value) || 0.175)}
                        /> 
                        rad
                    </label>
                    <label>
                        Glideslope k: 
                        <input type="number" value={glideslopeK} step="0.0001" min="0.0000"
                            onChange={e => setGlideslopeK(parseFloat(e.target.value) || 0.001)}
                        />
                    </label>
                    <label>
                        Minimum Range:
                        <input type="numner" value={minRange} step="1" min="0"
                            onChange={e => setMinRange(parseFloat(e.target.value) || 10)}
                        /> 
                        m
                    </label>
                </div>

                <h4>LQR Weights</h4>
                <div className="input-row">
                    <label>
                        Q_pos:
                        <input type="range" value={qPos} min="1" max="500" step="1"
                            onChange={e => setQPos(parseFloat(e.target.value))}
                        />
                        <span className="value">{qPos}</span>
                    </label>
                    <label>
                        Q_vel:
                        <input type="range" value={qVel} min="1" max="500" step="1"
                            onChange={e => setQVel(parseFloat(e.target.value))}
                        />
                        <span className="value">{qVel}</span>
                    </label>
                </div>
                <div className="input-row">
                    <label>
                        R:
                        <input type="range" value={R} min="0.01" max="100" step="0.1"
                            onChange={e => setR(parseFloat(e.target.value))}
                        />
                        <span className="value">{R}</span>
                    </label>
                    <label>
                        u_max:
                            <input type="number" value={uMax} step="0.001" min="0.001"
                                onChange={e => setUMax(parseFloat(e.target.value) || 0.01)}
                            />
                    </label>
                </div>

                <h4>Simulation</h4>
                <div className="input-row">
                    <label>
                        dt:
                        <input type="number" value={dt} step="0.5" min="0.1"
                            onChange={e => setDt(parseFloat(e.target.value) || 1)}
                        /> 
                        s
                    </label>
                    <label>
                        Timeout:
                        <input type="number" value={timeout} step="1" min="1"
                            onChange={e => setTimeout_(parseFloat(e.target.value) || 6000.0)}
                        />
                        s
                    </label>
                    <label>
                        Success Range:
                            <input type="number" value={successRange} step="1" min="1"
                                onChange={e => setSuccessRange(parseFloat(e.target.value) || 5)}
                            />
                            m
                    </label>
                    <label>
                        Success Velocity:
                            <input type="number" value={successVelocity} step="0.01" min="0.01"
                                onChange={e => setSuccessVelocity(parseFloat(e.target.value) || 0.05)}
                            />
                            m/s
                    </label>
                </div>
            </div>

            <button className="run-btn" onClick={run} disabled={loading}>
                {loading ? 'Running guidance...' : 'Run Approach'}
            </button>

            {error && <div className="error-msg">{error}</div>}

            {result && (
                <>
                    {/* Metrics */}
                    <div className="metric-row">
                        <div className={`metric ${result.success ? 'highlight-ok' : 'highlight-err'}`}>
                            <span className="metric-label">Result</span>
                            <span className="metric-value">{result.success ? 'SUCCESS' : 'FAILED'}</span>
                        </div>
                        <div className="metric">
                            <span className="metric-label">Total dv</span>
                            <span className="metric-value">{result.totalDV.toFixed(3)} m/s</span>
                        </div>
                        <div className="metric">
                            <span className="metric-label">Final Range</span>
                            <span className="metric-value">{result.finalRange.toFixed(2)} m</span>
                        </div>
                        <div className="metric">
                            <span className="metric-label">Final Velocity</span>
                            <span className="metric-value">{result.finalVelocity.toFixed(2)} m/s</span>
                        </div>
                        <div className="metric">
                            <span className="metric-label">Duration</span>
                            <span className="metric-value">{result.duration.toFixed(3)} s</span>
                        </div>
                        <div className="metric">
                            <span className="metric-label">Saturations</span>
                            <span className="metric-value">{result.saturationCount}</span>
                        </div>
                    </div>

                    {/* Plots */}
                    <div className="plot-grid">
                        <div className="plot-container">
                            <Plot
                                data={trajTraces}
                                layout={dark3DLayout({
                                    title: { text: 'Approach Trajectory (LVLH)', font: { color: '#c8c8c8' } },
                                })}
                                config={PLOTLY_CONFIG}
                                style={{ width: '100%', height: '500px' }}
                            />
                        </div>
                        <div className="plot-container">
                            <Plot
                                data={rangeTraces}
                                layout={darkLayout({
                                    title: { text: 'Range to Target', font: { color: '#c8c8c8' } },
                                    xaxis: { title: 'Time [s]' },
                                    yaxis: { title: 'Range [m]' },
                                })}
                                config={PLOTLY_CONFIG}
                                style={{ width: '100%', height: '350px' }}
                            />
                        </div>
                        <div className="plot-container">
                            <Plot
                                data={ctrlTraces}
                                layout={darkLayout({
                                    title: { text: 'Control Magnitude', font: { color: '#c8c8c8' }},
                                    xaxis: { title: 'Time [s]' },
                                    yaxis: { title: '|u| [m/s^2]' },
                                })}
                                config={PLOTLY_CONFIG}
                                style={{ width: '100%', height: '350px' }}
                            />
                        </div>
                    </div>
                </>
            )}
        </div>
    );
}

export default GuidancePanel;