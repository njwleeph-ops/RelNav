import React, { useState } from 'react';
import Plot from 'react-plotly.js';
import { runApproachGuidance } from '../api';
import { COLORS, PLOTLY_CONFIG, layout3D, getAxisConstraint, getDefaultState } from '../constants';
import StateInput from './StateInput';

function GuidancePanel({ config, savedResult, onResult }) {
    const lockedAxis = config ? getAxisConstraint(config.glideslope?.approach_axis) : 'y';
    const [x0, setX0] = useState(config ? getDefaultState(config.glideslope?.approach_axis) : getDefaultState(null));
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState(null);

    const result = savedResult;

    const run = async () => {
        setLoading(true);
        setError(null);
        try {
            const data = await runApproachGuidance({
                x: x0.x, y: x0.y, z: x0.z,
                vx: x0.vx, vy: x0.vy, vz: x0.vz,
            });
            if (data.error) throw new Error(data.error);
            onResult(data);
        } catch (e) {
            setError(e.message);
        } finally {
            setLoading(false);
        }
    };

    // Build 3D traces
    const traces = [];

    if (result?.trajectory) {
        const traj = result.trajectory;

        // Trajectory line
        traces.push({
            x: traj.map(p => p.x),
            y: traj.map(p => p.y),
            z: traj.map(p => p.z),
            mode: 'lines',
            type: 'scatter3d',
            name: 'Trajectory',
            line: { color: COLORS.trajectory, width: 3 },
        });

        // Initial state marker
        traces.push({
            x: [traj[0].x],
            y: [traj[0].y],
            z: [traj[0].z],
            mode: 'markers',
            type: 'scatter3d',
            name: 'Start',
            marker: { color: COLORS.trajectory, size: 6, symbol: 'diamond' },
        });

        // Final state marker
        const last = traj[traj.length - 1];
        traces.push({
            x: [last.x],
            y: [last.y],
            z: [last.z],
            mode: 'markers',
            type: 'scatter3d',
            name: 'End',
            marker: { color: result.success ? COLORS.success : COLORS.failure, size: 6 },
        });

        // Target at origin
        traces.push({
            x: [0], y: [0], z: [0],
            mode: 'markers',
            type: 'scatter3d',
            name: 'Target',
            marker: { color: COLORS.target, size: 5, symbol: 'cross' },
        });
    }

    // Corridor cone mesh
    if (result?.corridorCone) {
        const verts = result.corridorCone.vertices;
        const faces = result.corridorCone.faces;

        traces.push({
            type: 'mesh3d',
            x: verts.map(v => v[0]),
            y: verts.map(v => v[1]),
            z: verts.map(v => v[2]),
            i: faces.map(f => f[0]),
            j: faces.map(f => f[1]),
            k: faces.map(f => f[2]),
            opacity: 0.15,
            color: '#6478b4',
            flatshading: true,
            name: 'Approach Corridor',
            showlegend: true,
            hoverinfo: 'skip',
        });
    }

    return (
        <div className="panel">
            <div className="panel-header">
                <h2>Single-Approach Guidance</h2>
            </div>

            <div className="input-section">
                <StateInput state={x0} onChange={setX0} lockedAxis={lockedAxis} />
                <button className="run-btn" onClick={run} disabled={loading}>
                    {loading ? 'Running...' : 'Run Approach'}
                </button>
            </div>

            {error && <div className="error-msg">{error}</div>}

            {result && (
                <div className="results-section">
                    <table className="results-table">
                        <thead>
                            <tr>
                                <th>Result</th>
                                <th>Total ΔV</th>
                                <th>Duration</th>
                                <th>Final Range</th>
                                <th>Final Velocity</th>
                                <th>Saturations</th>
                                <th>Measurements</th>
                                <th>Dropouts</th>
                            </tr>
                        </thead>
                        <tbody>
                            <tr>
                                <td className={result.success ? 'val-ok' : 'val-err'}>
                                    {result.success ? 'SUCCESS' : 'FAILED'}
                                </td>
                                <td>{result.totalDV.toFixed(4)} m/s</td>
                                <td>{result.duration.toFixed(1)} s</td>
                                <td>{result.finalRange.toFixed(3)} m</td>
                                <td>{result.finalVelocity.toFixed(4)} m/s</td>
                                <td>{result.saturationCount}</td>
                                <td>{result.measurementCount}</td>
                                <td>{result.dropoutCount}</td>
                            </tr>
                        </tbody>
                    </table>

                    <div className="plot-container">
                        <Plot
                            data={traces}
                            layout={layout3D({ title: { text: 'Approach Trajectory (LVLH)' } })}
                            config={PLOTLY_CONFIG}
                            style={{ width: '100%', height: '550px' }}
                        />
                    </div>
                </div>
            )}
        </div>
    );
}

export default GuidancePanel;