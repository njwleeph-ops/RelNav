import React, { useState } from 'react';
import Plot from 'react-plotly.js';
import { runMonteCarlo, createState, createPosition } from '../api';
import { ISS_ALTITUDE, darkLayout, dark3DLayout, PLOTLY_CONFIG, COLOR } from '../constants';

function MonteCarloPanel() {
    // --- Initial state ---
    const [x0, setX0] = useState({ x: 200, y: -500, z: 0, vx: 0, vy: 0; vz: 0 });

    // --- Corridor Parameters ---
    const [approachAxis, setApproachAxis] = useState('vbar');
    const [corridorAngle, setCorridorAngle] = useState(0.175);
    const [glideslopeK, setGlideslopeK] = useState(0.001);
    const [minRange, setMinRange] = useState(10.0);

    // --- LQR weights ---
    const [qPos, setQPos] = useState(10);
    const [qVel, setQVel] = useState(50);
    const [R, setR] = useState(1.0);
    const [uMax, setUMax] = useState(0.01);

    // --- Simulation ---
    const [dt, setDt] = useState(1.0);
    const [timeout, setTimeout_] = useState(6000.0);
    const [successRange, setSuccessRange] = useState(5.0);
    const [successVelocity, setSuccessVelocity] = useState(0.05);

    // --- Uncertainty model ---
    const [posErr, setPosErr] = useState(5);
    const [velErr, setVelErr] = useState(0.01);
    const [thrustMagErr, setThrustMagErr] = useState(0.02);
    const [thrustPtErr, setThrustPtErr] = useState(0.005);

    // --- MC Config --- 
    const [nSamples, setNSamples] = useState(500);
    const [seed, setSeed] = useState(42);

    // --- Results ---
    const [loading, setLoading] = useState(false);
    const [result, setResult] = useState(null);
    const [error, setError] = useState(null);

    const run = async () => {
        setLoading(true);
        setError(null);
        try {
            const data = await runMonteCarlo({
                initialState: createState(x0.x, x0.y, x0.z, x0.vx, x0.vy, x0.vz),
                altitude: ISS_ALTITUDE,
                nSamples,
                seed,
                corridor: {
                    axis: { vbar: { x: 0, y: -1, z: 0 }, rbar: { x: -1, y: 0, z: 0}, hbar: { x: 0, y: 0, z: -1 } }[approachAxis],
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
                uncertainty: {
                    posError: createPosition(posErr, posErr, posErr),
                    velError: createPosition(velErr, velErr, velErr),
                    thrustMagError: thrustMagErr,
                    thrustPointingError: thrustPtErr,
                },
            });
            setResult(data);
        } catch (e) {
            setError(e.message);
        } finally {
            setLoading(false);
        }
    };

    // --- Final position scatter plot ---
    const scatterTraces = [];
    
    if (result?.finalStates) {
        const success = result.finalStates.filter((_, i) =>
            result.samples[i] && result.samples[i].success
        );
        const failed = result.finalStates.filter((_, i) =>
            result.samples[i] && !result.samples[i].success
        );

        if (success.length > 0) {
            scatterTraces.push({
                x: success.map(s => s.y), 
                y: success.map(s => s.x), 
                z: success.map(s => s.z),
                mode: 'markers',
                name: 'success',
                type: 'scatter3d',
                marker: { color: COLORS.green, size: 2, opacity: 0.5 },
            });
        }

        if (failed.length > 0) {
            scatterTraces.push({
                x: failed.map(s => s.y),
                y: failed.map(s => s.x),
                z: failed.map(s => s.z),
                mode: 'markers',
                name: 'success',
                type: 'scatter3d',
                marker: { color: COLORS.red, size: 2, opacity: 0.5 },
            });
        }

        // Success range
        const theta = Array.from({ length: 64 }, (_, i) => (i / 63) * 2 * Math.PI);
        const r = successRange;
        scatterTraces.push({
            x: theta.map(t => r * Math.cos(t)),
            y: theta.map(t => r * Math.sin(t)),
            z: theta.map(() => 0),
            mode: 'lines',
            name: `Success (${r} m)`,
            type: 'scatter3d',
            line: { color: COLORS.amber, width: 2, dash: 'dash' },
        });
        scatterTraces.push({
            x: theta.map(t => r * Math.cos(t)),
            y: theta.map(() => 0);
            z: theta.map(t => r * Math.sin(t)),
            mode: 'lines',
            showlegend: false,
            type: 'scatter3d',
            line: { color: COLORS.amber, width: 1, dash: 'dash'},
        });
        
        // Target
        scatterTraces.push({
            x: [0],
            y: [0],
            z: [0],
            mode: 'markers',
            name: 'Target',
            type: 'scatter3d',
            marker: { color: COLORS.amber, size: 5, symbol: 'cross' },
        });
    }

    // dv histogram
    const dvTraces = [];
    
    if (result?.samples) {
        dvTraces.push({
            x: result.samples.map(s => s.totalDV),
            type: 'histogram',
            name: 'dv Distribution',
            marker: { color: COLORS.cyan },
            opacity: 0.75,
        });
    }

    // --- Duration histogram ---
    const durTraces = [];

    if (result?.samples) {
        durTraces.push({
            x: result.samples.map(s => s.duration),
            type: 'histogram',
            name: 'Duration Distribution',
            marker: { color: COLORS.magenta },
            opacity: 0.75,
        });
    }

    return (
        <div className="panel">
            <div className="panel-header">
                <h2>Monte Carlo Analysis</h2>
                <p>Dispersed approach guidance under nav/thruster uncertainties</p>
            </div>

            <div className="input-section">
                <h4>Initial State (LVLH)</h4>
                <div className="input-row">
                    {['x', 'y', 'z'].map(k => (
                        <label key={k}>
                            {k}:
                            <input type="number" value={x0[k]}
                                onChange={e => setX0({ ...x0, [k]: parseFloat(e.target.value) || 0})}
                            />
                            m
                        </label>
                    ))}
                </div>
                <div className="input-row">
                    {['vx', 'vy', 'vz'].map(k => (
                        <label key={k}>
                            {k}:
                            <input 
                                type="number"
                                value={x0[k]}
                                step="0.01"
                                onChange={e => setX0({ ...x0, [k]: parseFloat(e.target.value) || 0})}
                            />
                            m/s
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
                        Corridor Angle (half):
                        <input type="number" value={corridorAngle} step="0.01"
                            onChange={e => setCorridorAngle(parseFloat(e.target.value) || 0.175)}
                        />
                        rad
                    </label>
                    <label>
                        Glideslope k:
                        <input type="number" value={glideslopeK} step="0.0005"
                            onChange={e => setGlideslopeK(parseFloat(e.target.value) || 0.001)}
                        />
                    </label>
                    <label>
                        Minimum Range:
                        <input type="number" value={minRange} step="0.5"
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
                    <label>
                        R:
                        <input type="range" value={R} min="0.01" max="100" step="0.1"
                            onChange={e => setR(parseFloat(e.target.value))}
                        />
                        <span className="value">{R}</span>
                    </label>
                </div>
                <div className="input-row">
                    <label>
                        u_max:
                        <input type="number" value={uMax} step="0.001"
                            onChange={e => setUMax(parseFloat(e.target.value) || 0.01)}
                        />
                        m/s^2
                    </label>
                    <label>
                        dt:
                        <input type="number" value={dt} step="0.5" min="0.1"
                            onChange={e => setDt(parseFloat(e.target.value) || 1.0)}
                        />
                        s
                    </label>
                    <label>
                        Timeout:
                        <input type="number" value={timeout} step="100"
                            onChange={e => setTimeout_(parseFloat(e.target.value) || 6000.0)}
                        />
                        s
                    </label>
                </div>
                <div className="input-row">
                    <label>
                        Success Range:
                        <input type="number" value={successRange} step="0.5"
                            onChange={e => setSuccessRange(parseFloat(e.target.value) || 5.0)}
                        />
                        m
                    </label>
                    <label>
                        Success Velocity:
                        <input type="number" value={successVelocity} step="0.01"
                            onChange={e => setSuccessVelocity(parseFloat(e.target.value) || 0.05)}
                        />
                        m/s
                    </label>
                </div>

                <h4>Uncertainty Model</h4>
                <div className="input-row">
                    <label>
                        Position:
                        <input type="number" value={posErr} step="0.5" min="0.0"
                            onChange={e => setPosErr(parseFloat(e.target.value) || 0.0)}
                        />
                        m
                    </label>
                    <label>
                        Velocity:
                        <input type="number" value={velErr} step="0.001" min="0.0"
                            onChange={e => setVelErr(parseFloat(e.target.value) || 0.0)}
                        />
                        m/s
                    </label>
                    <label>
                        Thrust Magnitude:
                        <input type="number" value={thrustMagErr} step="0.005" min="0.0"
                            onChange={e => setThrustMagErr(parseFloat(e.target.value) || 0.0)}
                        />
                    </label>
                    <label>
                        Thrust Pointing:
                        <input type="number" value={thrustPtErr} step="0.001" min="0.0"
                            onChange={e => setThrustPtErr(parseFloat(e.target.value) || 0.0)}
                        />
                    </label>
                </div>

                <h4>Monte Carlo Config</h4>
                <div className="input-row">
                    <label>
                        Samples:
                        <input type="number" value={nSamples} step="100" min="50"
                            onChange={e => setNSamples(parseInt(e.target.value) || 500)}
                        />
                    </label>
                    <label>
                        Seed:
                        <input type="number" value={seed} min="0"
                            onChange={e => setSeed(parseInt(e.target.value) || 42)}
                        />
                    </label>
                </div>
            </div>

            <button className="run-btn" onClick={run} disabled={loading}>
                {loading ? 'Running MC...' : 'Run Monte Carlo'}
            </button>

            {error && <div className="error-msg">{error}</div>}

            {result && (
                <>
                    {/* Key metrics */}
                    <div className="metric-row">
                        <div className={`metric ${result.successRate > 0.9 ? 'highlight-ok' : result.successRate > 0.5 ? 'highlight-warn' : 'highlight-err'}`}>
                            <span className="metric-label">Success Rate</span>
                            <span className="metric-value">{(result.successRate * 100).toFixed(1)}%</span>
                        </div>
                        <div className="metric">
                            <span className="metric-label">Samples</span>
                            <span className="metric-value">{result.nSuccess} / {result.nSamples}</span>
                        </div>
                        <div className="metric">
                            <span className="metric-label">Mean dv</span>
                            <span className="metric-vaue">{result.meanDV.toFixed(3)} ± {result.stdDV.toFixed(3)} m/s</span>
                        </div>
                        <div className="metric">
                            <span className="metric-label">Mean Duration</span>
                            <span className="metric-value">{result.meanDuration.toFixed(0)} ± {result.stdDuration.toFixed(0)} s</span>
                        </div>
                        <div className="metric">
                            <span className="metric-label">Mean Final Range</span>
                            <span className="metric-value">{result.meanFinalRange.toFixed(2)} m</span>
                        </div>
                        <div className="metric">
                            <span className="metric-label">Mean Saturations</span>
                            <span className="metric-value">{result.meanSaturationCount.toFixed(1)}</span>
                        </div>
                    </div>

                    {/* Plots */}
                    <div className="plot-grid">
                        <div className="plot-container">
                            <Plot
                                data={scatterTraces}
                                layout={dark3DLayout({
                                    title: { text: 'Final Position Scatter (LVLH)', font: { color: '#c8c8d8' } },
                                })}
                                config={PLOTLY_CONFIG}
                                style={{ width: '100%', height: '500px' }}
                            />
                        </div>
                        <div className="plot-container">
                            <Plot
                                data={dvTraces}
                                layout={darkLayout({
                                    title: { text: 'dv Distribution', font: { color: '#c8c8d8' } },
                                    xaxis: { title: 'Total dv [m/s' },
                                    yaxis: { title: 'Count' },
                                    showlegend: false,
                                })}
                                config={PLOTLY_CONFIG}
                                style={{ width: '100%', height: '350px' }}
                            />
                        </div>
                        <div className="plot-container">
                            <Plot
                                data={durTraces}
                                layout={darkLayout({
                                    title: { text: 'Duration Distribution', font: { color: '#c8c8d8' } },
                                    xaxis: { title: 'Duration [s]' },
                                    yaxis: { title: 'Count' },
                                    showlegend: false,
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

export default MonteCarloPanel;