import React, { useState } from 'react';
import Plot from 'react-plotly.js';
import { propagateTrajectory, validatePropagators } from '../api';

const ISS_PERIOD = 5569.4;
const ISS_ALTITUDE = 420e3;

const PRESETS = {
    football: { x: 1000, y: 0, z: 0, vx: 0, vy: 0, vz: 0, label: 'Football (Radial)' },
    teardrop: { x: 0, y: -1000, z: 0, vx: 0.5, vy: 0, vz: 0, label: 'Teardrop' },
    vbar_hop: { x: 0, y: -500, z: 0, vx: 0, vy: 0.2, vz: 0, label: 'V-bar Hop' },
    cross_track: { x: 0, y: 0, z: 500, vx: 0, vy: 0, vz: 0, label: 'Cross-track' },
    custom: { x: 0, y: 0, z: 0, vx: 0, vy: 0, vz: 0, label: 'Custom' }
};

function PropagatorPanel() {
    const [loading, setLoading] = useState(false);
    const [trajectory, setTrajectory] = useState(null);
    const [validation, setValidation] = useState(null);
    const [error, setError] = useState(null);

    const [preset, setPreset] = useState('football');
    const [state, setState] = useState(PRESETS.football);
    const [duration, setDuration] = useState(2);

    const handlePresetChange = (e) => {
        const key = e.target.value;
        setPreset(key);
        if (key !== 'custom') {
            setState(PRESETS[key]);
        }
    };

    const handleStateChange = (field, value) => {
        setPreset('custom');
        setState(prev => ({ ...prev, [field]: parseFloat(value) || 0 }));
    };

    const runPropagator = async () => {
        setLoading(true);
        setError(null);

        try {
            const initialState = {
                x: state.x, y: state.y, z: state.z,
                vx: state.vx, vy: state.vy, vz: state.vz
            };

            const [trajResult, validResult] = await Promise.all([
                propagateTrajectory({
                    initialState,
                    duration: duration * ISS_PERIOD,
                    altitude: ISS_ALTITUDE,
                    method: 'analytical',
                    numPoints: 500
                }),
                validatePropagators({
                    initialState,
                    duration: duration * ISS_PERIOD,
                    altitude: ISS_ALTITUDE
                })
            ]);

            setTrajectory(trajResult);
            setValidation(validResult);
        } catch (err) {
            setError(err.message);
        } finally {
            setLoading(false);
        }
    };

    return (
        <div className="panel">
            <div className="panel-header">
                <h2>Clohessy-Wiltshire Propagator</h2>
                <p>Free-drift relative motion in the LVLH frame.</p>
            </div>

            <div className="input-section">
                <div className="input-row">
                    <label>
                        Preset:
                        <select value={preset} onChange={handlePresetChange}>
                            {Object.entries(PRESETS).map(([key, val]) => (
                                <option key={key} value={key}>{val.label}</option>
                            ))}
                        </select>
                    </label>
                    <label>
                        Duration:
                        <input
                            type="number"
                            value={duration}
                            onChange={(e) => setDuration(parseFloat(e.target.value) || 1)}
                            min={0.1}
                            max={10}
                            step={0.5}
                        />
                        <span className="unit">orbits</span>
                    </label>
                </div>

                <div className="input-row">
                    <label>x: <input type="number" value={state.x} onChange={(e) => handleStateChange('x', e.target.value)} /> m</label>
                    <label>y: <input type="number" value={state.y} onChange={(e) => handleStateChange('y', e.target.value)} /> m</label>
                    <label>z: <input type="number" value={state.z} onChange={(e) => handleStateChange('z', e.target.value)} /> m</label>
                </div>
                <div className="input-row">
                    <label>vx: <input type="number" value={state.vx} onChange={(e) => handleStateChange('vx', e.target.value)} step="0.1" /> m/s</label>
                    <label>vy: <input type="number" value={state.vy} onChange={(e) => handleStateChange('vy', e.target.value)} step="0.1" /> m/s</label>
                    <label>vz: <input type="number" value={state.vz} onChange={(e) => handleStateChange('vz', e.target.value)} step="0.1" /> m/s</label>
                </div>
            </div>

            <button className="run-btn" onClick={runPropagator} disabled={loading}>
                {loading ? 'Running...' : 'Propagate'}
            </button>

            {error && <div className="error-msg">{error}</div>}

            {validation && (
                <div className="results-card">
                    <h3>Propagator Validation</h3>
                    <div className="result-row">
                        <span>Max Position Error (RK4 vs STM):</span>
                        <span className="value">{validation.maxPositionError.toExponential(2)} m</span>
                    </div>
                    <div className="result-row">
                        <span>Max Velocity Error:</span>
                        <span className="value">{validation.maxVelocityError.toExponential(2)} m/s</span>
                    </div>
                </div>
            )}

            {trajectory && (
                <div className="plot-container">
                    <Plot
                        data={[
                            {
                                x: trajectory.trajectory.map(p => p.y),
                                y: trajectory.trajectory.map(p => p.x),
                                mode: 'lines',
                                name: 'Trajectory',
                                line: { color: '#00d4ff', width: 2 }
                            },
                            {
                                x: [0],
                                y: [0],
                                mode: 'markers',
                                name: 'Target',
                                marker: { color: '#ff6b6b', size: 14, symbol: 'star' }
                            },
                            {
                                x: [trajectory.trajectory[0].y],
                                y: [trajectory.trajectory[0].x],
                                mode: 'markers',
                                name: 'Start',
                                marker: { color: '#4ecdc4', size: 10 }
                            }
                        ]}
                        layout={{
                            title: { text: 'CW Relative Motion (R-V Plane)', font: { color: '#e0e0e0' } },
                            xaxis: { title: 'V-bar (Along-track) [m]', color: '#a0a0a0', gridcolor: '#333', zerolinecolor: '#555' },
                            yaxis: { title: 'R-bar (Radial) [m]', color: '#a0a0a0', gridcolor: '#333', zerolinecolor: '#555', scaleanchor: 'x' },
                            paper_bgcolor: '#1a1a2e',
                            plot_bgcolor: '#1a1a2e',
                            font: { color: '#e0e0e0' },
                            showlegend: true,
                            legend: { font: { color: '#a0a0a0' } }
                        }}
                        config={{ responsive: true }}
                        style={{ width: '100%', height: '500px' }}
                    />
                </div>
            )}
        </div>
    );
}

export default PropagatorPanel;