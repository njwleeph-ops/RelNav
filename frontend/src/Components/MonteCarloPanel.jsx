import React, { useState } from 'react';
import Plot from 'react-plotly.js';
import { runTargetedMonteCarlo } from '../api';

const ISS_PERIOD = 5569.4;
const ISS_ALTITUDE = 420e3;

function MonteCarloPanel() {
    const [loading, setLoading] = useState(false);
    const [result, setResult] = useState(null);
    const [error, setError] = useState(null);

    // Initial state
    const [x0, setX0] = useState({ x: 200, y: -1000, z: 0, vx: 0, vy: 0, vz: 0 });

    // Target
    const [target, setTarget] = useState({ x: 0, y: 0, z: 0 });

    // Time of flight
    const [tofOrbits, setTofOrbits] = useState(0.25);

    // Samples
    const [nSamples, setNSamples] = useState(1000);

    // Uncertainties
    const [posError, setPosError] = useState(5);
    const [velError, setVelError] = useState(0.01);
    const [thrustMagError, setThrustMagError] = useState(0.03);
    const [thrustPointError, setThrustPointError] = useState(0.01);

    // Corridor
    const [corridorRadius, setCorridorRadius] = useState(100);

    const runAnalysis = async () => {
        setLoading(true);
        setError(null);

        try {
            const mcResult = await runTargetedMonteCarlo({
                initialState: x0,
                target: target,
                timeOfFlight: tofOrbits * ISS_PERIOD,
                altitude: ISS_ALTITUDE,
                nSamples: nSamples,
                uncertainty: {
                    posError: { x: posError, y: posError, z: posError },
                    velError: { x: velError, y: velError, z: velError },
                    thrustMagError: thrustMagError,
                    thrustPointingError: thrustPointError
                },
                corridor: {
                    type: 'cylinder',
                    radius: corridorRadius
                }
            });
            setResult(mcResult);
        } catch (err) {
            setError(err.message);
        } finally {
            setLoading(false);
        }
    };

    return (
        <div className="panel">
            <div className="panel-header">
                <h2>Monte Carlo Dispersion Analysis</h2>
                <p>Targeted transfer with nav/thruster uncertainties. What's the probability of hitting the approach corridor?</p>
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

                <h4>Transfer & Samples</h4>
                <div className="input-row">
                    <label>TOF: <input type="number" value={tofOrbits} onChange={(e) => setTofOrbits(parseFloat(e.target.value) || 0.25)} step="0.05" min="0.05" max="2" /> orbits</label>
                    <label>Samples: <input type="number" value={nSamples} onChange={(e) => setNSamples(parseInt(e.target.value) || 1000)} min="100" max="10000" /></label>
                    <label>Corridor: <input type="number" value={corridorRadius} onChange={(e) => setCorridorRadius(parseFloat(e.target.value) || 100)} /> m</label>
                </div>

                <h4>Uncertainties (1σ)</h4>
                <div className="input-row">
                    <label>Position: <input type="number" value={posError} onChange={(e) => setPosError(parseFloat(e.target.value) || 0)} step="1" /> m</label>
                    <label>Velocity: <input type="number" value={velError} onChange={(e) => setVelError(parseFloat(e.target.value) || 0)} step="0.001" /> m/s</label>
                </div>
                <div className="input-row">
                    <label>Thrust Mag: <input type="number" value={thrustMagError} onChange={(e) => setThrustMagError(parseFloat(e.target.value) || 0)} step="0.01" /> (3%=0.03)</label>
                    <label>Thrust Point: <input type="number" value={thrustPointError} onChange={(e) => setThrustPointError(parseFloat(e.target.value) || 0)} step="0.005" /> rad</label>
                </div>
            </div>

            <button className="run-btn" onClick={runAnalysis} disabled={loading}>
                {loading ? 'Running...' : `Run Monte Carlo (N=${nSamples})`}
            </button>

            {error && <div className="error-msg">{error}</div>}

            {result && (
                <>
                    <div className="results-card">
                        <h3>Results</h3>
                        <div className="result-row">
                            <span>Nominal ΔV:</span>
                            <span className="value">{result.nominalDV.totalDV.toFixed(3)} m/s</span>
                        </div>
                        <div className="result-row">
                            <span>Mean Arrival:</span>
                            <span className="value">[{result.meanArrival.x.toFixed(1)}, {result.meanArrival.y.toFixed(1)}, {result.meanArrival.z.toFixed(1)}] m</span>
                        </div>
                        <div className="result-row">
                            <span>3σ Dispersion:</span>
                            <span className="value">[{result.sigma3Arrival.x.toFixed(1)}, {result.sigma3Arrival.y.toFixed(1)}, {result.sigma3Arrival.z.toFixed(1)}] m</span>
                        </div>
                        <div className="result-row highlight">
                            <span>Corridor Success Rate:</span>
                            <span className="value">{(result.corridorSuccessRate * 100).toFixed(1)}%</span>
                        </div>
                    </div>

                    <div className="plot-container">
                        <Plot
                            data={[
                                {
                                    x: result.finalPositions.filter((_, i) => result.insideCorridor[i]).map(p => p.y),
                                    y: result.finalPositions.filter((_, i) => result.insideCorridor[i]).map(p => p.x),
                                    mode: 'markers',
                                    name: 'Inside Corridor',
                                    marker: { color: '#4ecdc4', size: 4 }
                                },
                                {
                                    x: result.finalPositions.filter((_, i) => !result.insideCorridor[i]).map(p => p.y),
                                    y: result.finalPositions.filter((_, i) => !result.insideCorridor[i]).map(p => p.x),
                                    mode: 'markers',
                                    name: 'Outside Corridor',
                                    marker: { color: 'rgba(255, 107, 107, 0.5)', size: 4 }
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
                                title: { text: `Arrival Dispersion (${(result.corridorSuccessRate * 100).toFixed(1)}% success)`, font: { color: '#e0e0e0' } },
                                xaxis: { title: 'V-bar [m]', color: '#a0a0a0', gridcolor: '#333', zerolinecolor: '#555' },
                                yaxis: { title: 'R-bar [m]', color: '#a0a0a0', gridcolor: '#333', zerolinecolor: '#555', scaleanchor: 'x' },
                                paper_bgcolor: '#1a1a2e',
                                plot_bgcolor: '#1a1a2e',
                                font: { color: '#e0e0e0' },
                                showlegend: true,
                                legend: { font: { color: '#a0a0a0' } },
                                shapes: [{
                                    type: 'circle',
                                    xref: 'x',
                                    yref: 'y',
                                    x0: target.y - corridorRadius,
                                    y0: target.x - corridorRadius,
                                    x1: target.y + corridorRadius,
                                    y1: target.x + corridorRadius,
                                    line: { color: '#ffcc00', width: 2, dash: 'dash' }
                                }]
                            }}
                            config={{ responsive: true }}
                            style={{ width: '100%', height: '500px' }}
                        />
                    </div>
                </>
            )}
        </div>
    );
}

export default MonteCarloPanel;