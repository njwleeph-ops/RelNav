import React, { useState } from 'react';
import Plot from 'react-plotly.js';
import { simulateLQRApproach, checkGlideslope } from '../api';

const ISS_PERIOD = 5569.4;
const ISS_ALTITUDE = 420e3;

function LQRPanel() {
    const [loading, setLoading] = useState(false);
    const [result, setResult] = useState(null);
    const [glideslopeResults, setGlideslopeResults] = useState([]);
    const [error, setError] = useState(null);

    // Initial state
    const [x0, setX0] = useState({ x: 100, y: -200, z: 50, vx: 0, vy: 0, vz: 0 });

    // Duration
    const [durationOrbits, setDurationOrbits] = useState(0.5);

    // LQR weights
    const [qPos, setQPos] = useState(100);
    const [qVel, setQVel] = useState(10);
    const [R, setR] = useState(1000);

    // Control limit
    const [uMax, setUMax] = useState(0.01);

    // Glideslope
    const [glideslopeK, setGlideslopeK] = useState(0.001);

    const runLQR = async () => {
        setLoading(true);
        setError(null);

        try {
            const lqrResult = await simulateLQRApproach({
                initialState: x0,
                duration: durationOrbits * ISS_PERIOD,
                altitude: ISS_ALTITUDE,
                uMax: uMax,
                dt: 10,
                Q: { pos: qPos, vel: qVel },
                R: R
            });
            setResult(lqrResult);

            // Check glideslope, skip points near origin
            const gsChecks = await Promise.all(
                lqrResult.trajectory
                    .filter(pt => Math.sqrt(pt.x ** 2 + pt.y ** 2 + pt.z ** 2) > 20)
                    .map(pt =>
                        checkGlideslope({
                            state: { x: pt.x, y: pt.y, z: pt.z, vx: pt.vx, vy: pt.vy, vz: pt.vz },
                            k: glideslopeK
                        })
                    )
            );
            setGlideslopeResults(gsChecks);

        } catch (err) {
            setError(err.message);
        } finally {
            setLoading(false);
        }
    };

    const vecMag = (v) => Math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2);
    const violationCount = glideslopeResults.filter(g => g.violated).length;

    return (
        <div className="panel">
            <div className="panel-header">
                <h2>LQR Optimal Control</h2>
                <p>Continuous-time LQR approach with glideslope constraint checking.</p>
            </div>

            <div className="input-section">
                <h4>Initial State</h4>
                <div className="input-row">
                    <label>x: <input type="number" value={x0.x} onChange={(e) => setX0({ ...x0, x: parseFloat(e.target.value) || 0 })} /> m</label>
                    <label>y: <input type="number" value={x0.y} onChange={(e) => setX0({ ...x0, y: parseFloat(e.target.value) || 0 })} /> m</label>
                    <label>z: <input type="number" value={x0.z} onChange={(e) => setX0({ ...x0, z: parseFloat(e.target.value) || 0 })} /> m</label>
                </div>

                <h4>LQR Weights</h4>
                <div className="input-row">
                    <label>
                        Q_pos:
                        <input type="range" value={qPos} onChange={(e) => setQPos(parseFloat(e.target.value))} min="1" max="1000" step="10" />
                        <span className="value">{qPos}</span>
                    </label>
                    <label>
                        Q_vel:
                        <input type="range" value={qVel} onChange={(e) => setQVel(parseFloat(e.target.value))} min="0.1" max="100" step="1" />
                        <span className="value">{qVel}</span>
                    </label>
                </div>
                <div className="input-row">
                    <label>
                        R:
                        <input type="range" value={R} onChange={(e) => setR(parseFloat(e.target.value))} min="10" max="10000" step="100" />
                        <span className="value">{R}</span>
                    </label>
                    <label>
                        u_max:
                        <input type="number" value={uMax} onChange={(e) => setUMax(parseFloat(e.target.value) || 0.01)} step="0.001" /> m/s²
                    </label>
                </div>

                <h4>Simulation</h4>
                <div className="input-row">
                    <label>Duration: <input type="number" value={durationOrbits} onChange={(e) => setDurationOrbits(parseFloat(e.target.value) || 0.5)} step="0.1" /> orbits</label>
                    <label>
                        Glideslope k:
                        <input type="number" value={glideslopeK} onChange={(e) => setGlideslopeK(parseFloat(e.target.value) || 0.001)} step="0.0005" />
                    </label>
                </div>
            </div>

            <button className="run-btn" onClick={runLQR} disabled={loading}>
                {loading ? 'Simulating...' : 'Simulate LQR Approach'}
            </button>

            {error && <div className="error-msg">{error}</div>}

            {result && (
                <>
                    <div className="results-card">
                        <h3>Results</h3>
                        <div className="result-row">
                            <span>Initial Position:</span>
                            <span className="value">[{result.trajectory[0].x.toFixed(1)}, {result.trajectory[0].y.toFixed(1)}, {result.trajectory[0].z.toFixed(1)}] m</span>
                        </div>
                        <div className="result-row">
                            <span>Final Position:</span>
                            <span className="value">[{result.trajectory[result.trajectory.length - 1].x.toFixed(1)}, {result.trajectory[result.trajectory.length - 1].y.toFixed(1)}, {result.trajectory[result.trajectory.length - 1].z.toFixed(1)}] m</span>
                        </div>
                        <div className="result-row">
                            <span>Total ΔV:</span>
                            <span className="value">{result.totalDV.toFixed(3)} m/s</span>
                        </div>
                        <div className="result-row">
                            <span>Glideslope Violations:</span>
                            <span className="value">{violationCount} / {glideslopeResults.length} points</span>
                        </div>
                    </div>

                    <div className="plots-row">
                        <div className="plot-container half">
                            <Plot
                                data={[
                                    {
                                        x: result.trajectory.map(p => p.y),
                                        y: result.trajectory.map(p => p.x),
                                        mode: 'lines',
                                        name: 'Trajectory',
                                        line: { color: '#00d4ff', width: 2 }
                                    },
                                    {
                                        x: [result.trajectory[0].y],
                                        y: [result.trajectory[0].x],
                                        mode: 'markers',
                                        name: 'Start',
                                        marker: { color: '#4ecdc4', size: 10 }
                                    },
                                    {
                                        x: [0],
                                        y: [0],
                                        mode: 'markers',
                                        name: 'Target',
                                        marker: { color: '#ff6b6b', size: 14, symbol: 'star' }
                                    }
                                ]}
                                layout={{
                                    title: { text: 'LQR Approach', font: { color: '#e0e0e0', size: 14 } },
                                    xaxis: { title: 'V-bar [m]', color: '#a0a0a0', gridcolor: '#333', zerolinecolor: '#555' },
                                    yaxis: { title: 'R-bar [m]', color: '#a0a0a0', gridcolor: '#333', zerolinecolor: '#555', scaleanchor: 'x' },
                                    paper_bgcolor: '#1a1a2e',
                                    plot_bgcolor: '#1a1a2e',
                                    font: { color: '#e0e0e0' },
                                    showlegend: true,
                                    legend: { font: { color: '#a0a0a0', size: 10 } },
                                    margin: { t: 40, r: 20 }
                                }}
                                config={{ responsive: true }}
                                style={{ width: '100%', height: '400px' }}
                            />
                        </div>

                        <div className="plot-container half">
                            <Plot
                                data={[
                                    {
                                        x: glideslopeResults.map((_, i) => i * 10 / 60),
                                        y: glideslopeResults.map(g => g.approachVelocity),
                                        mode: 'lines',
                                        name: 'V_approach',
                                        line: { color: '#00d4ff' }
                                    },
                                    {
                                        x: glideslopeResults.map((_, i) => i * 10 / 60),
                                        y: glideslopeResults.map(g => g.maxVelocity),
                                        mode: 'lines',
                                        name: 'V_max (limit)',
                                        line: { color: '#ff6b6b', dash: 'dash' }
                                    }
                                ]}
                                layout={{
                                    title: { text: 'Glideslope Constraint', font: { color: '#e0e0e0', size: 14 } },
                                    xaxis: { title: 'Time [min]', color: '#a0a0a0', gridcolor: '#333' },
                                    yaxis: { title: 'Velocity [m/s]', color: '#a0a0a0', gridcolor: '#333' },
                                    paper_bgcolor: '#1a1a2e',
                                    plot_bgcolor: '#1a1a2e',
                                    font: { color: '#e0e0e0' },
                                    showlegend: true,
                                    legend: { font: { color: '#a0a0a0', size: 10 } },
                                    margin: { t: 40, r: 20 }
                                }}
                                config={{ responsive: true }}
                                style={{ width: '100%', height: '400px' }}
                            />
                        </div>
                    </div>
                </>
            )}
        </div>
    );
}

export default LQRPanel;