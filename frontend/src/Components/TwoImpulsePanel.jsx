import React, { useState } from 'react';
import Plot from 'react-plotly.js';
import { simulateTwoImpulse, getDVSweep } from '../api';

const ISS_PERIOD = 5569.4;
const ISS_ALTITUDE = 420e3;

function TwoImpulsePanel() {
    const [loading, setLoading] = useState(false);
    const [result, setResult] = useState(null);
    const [sweep, setSweep] = useState(null);
    const [error, setError] = useState(null);

    // Initial state
    const [x0, setX0] = useState({ x: 200, y: -1000, z: 0, vx: 0, vy: 0, vz: 0 });

    // Target
    const [target, setTarget] = useState({ x: 0, y: 0, z: 0 });

    // TOF
    const [tofOrbits, setTofOrbits] = useState(0.25);

    const runTransfer = async () => {
        setLoading(true);
        setError(null);

        try {
            const [transferResult, sweepResult] = await Promise.all([
                simulateTwoImpulse({
                    initialState: x0,
                    targetPosition: target,
                    timeOfFlight: tofOrbits * ISS_PERIOD,
                    altitude: ISS_ALTITUDE,
                    numPoints: 200
                }),
                getDVSweep({
                    initialState: x0,
                    targetPosition: target,
                    tofMin: 0.1 * ISS_PERIOD,
                    tofMax: 1.0 * ISS_PERIOD,
                    numPoints: 50,
                    altitude: ISS_ALTITUDE
                })
            ]);
            setResult(transferResult);
            setSweep(sweepResult);
        } catch (err) {
            setError(err.message);
        } finally {
            setLoading(false);
        }
    };

    const vecMag = (v) => Math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2);

    return (
        <div className="panel">
            <div className="panel-header">
                <h2>Two-Impulse Rendezvous</h2>
                <p>CW Lambert solver: compute ΔV pair for transfer from start to target.</p>
            </div>

            <div className="input-section">
                <h4>Initial State</h4>
                <div className="input-row">
                    <label>x: <input type="number" value={x0.x} onChange={(e) => setX0({ ...x0, x: parseFloat(e.target.value) || 0 })} /> m</label>
                    <label>y: <input type="number" value={x0.y} onChange={(e) => setX0({ ...x0, y: parseFloat(e.target.value) || 0 })} /> m</label>
                    <label>z: <input type="number" value={x0.z} onChange={(e) => setX0({ ...x0, z: parseFloat(e.target.value) || 0 })} /> m</label>
                </div>

                <h4>Target Position</h4>
                <div className="input-row">
                    <label>x: <input type="number" value={target.x} onChange={(e) => setTarget({ ...target, x: parseFloat(e.target.value) || 0 })} /> m</label>
                    <label>y: <input type="number" value={target.y} onChange={(e) => setTarget({ ...target, y: parseFloat(e.target.value) || 0 })} /> m</label>
                    <label>z: <input type="number" value={target.z} onChange={(e) => setTarget({ ...target, z: parseFloat(e.target.value) || 0 })} /> m</label>
                </div>

                <h4>Time of Flight</h4>
                <div className="input-row">
                    <label>
                        TOF:
                        <input
                            type="range"
                            value={tofOrbits}
                            onChange={(e) => setTofOrbits(parseFloat(e.target.value))}
                            min="0.05"
                            max="1.0"
                            step="0.01"
                        />
                        <span className="value">{tofOrbits.toFixed(2)} orbits ({(tofOrbits * ISS_PERIOD / 60).toFixed(1)} min)</span>
                    </label>
                </div>
            </div>

            <button className="run-btn" onClick={runTransfer} disabled={loading}>
                {loading ? 'Computing...' : 'Compute Transfer'}
            </button>

            {error && <div className="error-msg">{error}</div>}

            {result && (
                <>
                    <div className="results-card">
                        <h3>Maneuver Results</h3>
                        <div className="result-row">
                            <span>ΔV₁:</span>
                            <span className="value">
                                [{result.dv1.x.toFixed(3)}, {result.dv1.y.toFixed(3)}, {result.dv1.z.toFixed(3)}] m/s
                                <span className="mag">|{vecMag(result.dv1).toFixed(3)}|</span>
                            </span>
                        </div>
                        <div className="result-row">
                            <span>ΔV₂:</span>
                            <span className="value">
                                [{result.dv2.x.toFixed(3)}, {result.dv2.y.toFixed(3)}, {result.dv2.z.toFixed(3)}] m/s
                                <span className="mag">|{vecMag(result.dv2).toFixed(3)}|</span>
                            </span>
                        </div>
                        <div className="result-row highlight">
                            <span>Total ΔV:</span>
                            <span className="value">{result.totalDV.toFixed(3)} m/s</span>
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
                                        name: 'Transfer',
                                        line: { color: '#00d4ff', width: 3 }
                                    },
                                    {
                                        x: [x0.y],
                                        y: [x0.x],
                                        mode: 'markers',
                                        name: 'Start',
                                        marker: { color: '#4ecdc4', size: 12 }
                                    },
                                    {
                                        x: [target.y],
                                        y: [target.x],
                                        mode: 'markers',
                                        name: 'Target',
                                        marker: { color: '#ff6b6b', size: 14, symbol: 'star' }
                                    }
                                ]}
                                layout={{
                                    title: { text: 'Transfer Trajectory', font: { color: '#e0e0e0', size: 14 } },
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

                        {sweep && (
                            <div className="plot-container half">
                                <Plot
                                    data={[
                                        {
                                            x: sweep.sweep.map(s => s.tof / 60),
                                            y: sweep.sweep.map(s => s.totalDV),
                                            mode: 'lines',
                                            name: 'Total ΔV',
                                            line: { color: '#00d4ff', width: 2 }
                                        },
                                        {
                                            x: [tofOrbits * ISS_PERIOD / 60],
                                            y: [result.totalDV],
                                            mode: 'markers',
                                            name: 'Selected',
                                            marker: { color: '#ff6b6b', size: 12 }
                                        }
                                    ]}
                                    layout={{
                                        title: { text: 'ΔV vs Time of Flight', font: { color: '#e0e0e0', size: 14 } },
                                        xaxis: { title: 'TOF [min]', color: '#a0a0a0', gridcolor: '#333' },
                                        yaxis: { title: 'Total ΔV [m/s]', color: '#a0a0a0', gridcolor: '#333' },
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
                        )}
                    </div>
                </>
            )}
        </div>
    );
}

export default TwoImpulsePanel;