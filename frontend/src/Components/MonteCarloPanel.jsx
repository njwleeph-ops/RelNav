import React, { useState, useEffect } from 'react';
import Plot from 'react-plotly.js';
import { runMonteCarlo } from '../api';
import { COLORS, PLOTLY_CONFIG, layout3D, layout2D, getAxisConstraint, getDefaultState, computeSymmetricRanges } from '../constants';
import StateInput from './StateInput';

function MonteCarloPanel({ config, savedResult, onResult, overrides, onOverridesConsumed }) {
    const lockedAxis = config ? getAxisConstraint(config.glideslope?.approach_axis) : 'y';
    const [x0, setX0] = useState(config ? getDefaultState(config.glideslope?.approach_axis) : getDefaultState(null));
    const [nSamples, setNSamples] = useState(500);
    const [posErr, setPosErr] = useState({ x: 10, y: 10, z: 10 });
    const [velErr, setVelErr] = useState({ x: 0.01, y: 0.01, z: 0.01 });
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState(null);

    const result = savedResult;

    const run = async (stateOverride) => {
        const state = stateOverride || x0;
        setLoading(true);
        setError(null);
        try {
            const data = await runMonteCarlo(
                { x: state.x, y: state.y, z: state.z, vx: state.vx, vy: state.vy, vz: state.vz },
                nSamples,
                posErr,
                velErr
            );
            if (data.error) throw new Error(data.error);
            onResult(data);
        } catch (e) {
            setError(e.message);
        } finally {
            setLoading(false);
        }
    };

    // Handle drill-into from envelope — set state and auto-run
    useEffect(() => {
        if (!overrides) return;
        if (overrides.range) {
            const axis = config?.glideslope?.approach_axis;
            let newState = getDefaultState(axis);
            if (axis && Math.abs(axis[1]) > 0.9) newState.y = -overrides.range;
            else if (axis && Math.abs(axis[0]) > 0.9) newState.x = -overrides.range;
            else if (axis && Math.abs(axis[2]) > 0.9) newState.z = -overrides.range;
            setX0(newState);
            run(newState);
        }
        if (onOverridesConsumed) onOverridesConsumed();
    }, [overrides, onOverridesConsumed, config]);

    // 3D scatter traces
    const scatterTraces = [];

    if (result?.finalStates && result?.samples) {
        const successStates = [];
        const failedStates = [];

        result.finalStates.forEach((s, i) => {
            if (result.samples[i]?.success) successStates.push(s);
            else failedStates.push(s);
        });

        if (successStates.length > 0) {
            scatterTraces.push({
                x: successStates.map(s => s.x),
                y: successStates.map(s => s.y),
                z: successStates.map(s => s.z),
                mode: 'markers', type: 'scatter3d',
                name: 'Success',
                marker: { color: COLORS.success, size: 2, opacity: 0.5 },
            });
        }

        if (failedStates.length > 0) {
            scatterTraces.push({
                x: failedStates.map(s => s.x),
                y: failedStates.map(s => s.y),
                z: failedStates.map(s => s.z),
                mode: 'markers', type: 'scatter3d',
                name: 'Failed',
                marker: { color: COLORS.failure, size: 2, opacity: 0.5 },
            });
        }

        // Target
        scatterTraces.push({
            x: [0], y: [0], z: [0],
            mode: 'markers', type: 'scatter3d',
            name: 'Target',
            marker: { color: COLORS.target, size: 5, symbol: 'cross' },
        });

        // Success sphere
        if (config?.sim?.success_range) {
            const r = config.sim.success_range;
            const theta = Array.from({ length: 64 }, (_, i) => (i / 63) * 2 * Math.PI);

            scatterTraces.push({
                x: theta.map(t => r * Math.cos(t)),
                y: theta.map(t => r * Math.sin(t)),
                z: theta.map(() => 0),
                mode: 'lines', type: 'scatter3d',
                name: `Success (${r} m)`,
                line: { color: COLORS.success, width: 1, dash: 'dash' },
            });
            scatterTraces.push({
                x: theta.map(t => r * Math.cos(t)),
                y: theta.map(() => 0),
                z: theta.map(t => r * Math.sin(t)),
                mode: 'lines', type: 'scatter3d',
                showlegend: false,
                line: { color: COLORS.success, width: 1, dash: 'dash' },
            });
        }

        // Corridor cone from response
        if (result.corridorCone) {
            const verts = result.corridorCone.vertices;
            const faces = result.corridorCone.faces;
            scatterTraces.push({
                type: 'mesh3d',
                x: verts.map(v => v[0]),
                y: verts.map(v => v[1]),
                z: verts.map(v => v[2]),
                i: faces.map(f => f[0]),
                j: faces.map(f => f[1]),
                k: faces.map(f => f[2]),
                opacity: 0.12,
                color: COLORS.cone,
                name: 'Corridor',
                hoverinfo: 'skip',
            });
        }
    }

    // Range for 3D plot
    const allPoints = result?.finalStates || [];
    const ranges3D = computeSymmetricRanges(allPoints);

    return (
        <div className="panel">
            <div className="panel-header">
                <h2>Monte Carlo Analysis</h2>
            </div>

            <div className="input-section">
                <StateInput state={x0} onChange={setX0} lockedAxis={lockedAxis} />

                <div className="input-row">
                    <div className="input-field">
                        <span className="field-label">Samples</span>
                        <input type="number" value={nSamples} min={50} step={100}
                            onChange={e => setNSamples(parseInt(e.target.value) || 500)} />
                    </div>
                </div>

                <label className="input-label">Position Dispersion 1-std [m]</label>
                <div className="input-row">
                    {['x', 'y', 'z'].map(k => (
                        <div key={k} className="input-field">
                            <span className="field-label">{k}</span>
                            <input type="number" value={posErr[k]} step={1} min={0}
                                onChange={e => setPosErr({ ...posErr, [k]: parseFloat(e.target.value) || 0 })} />
                            <span className="field-unit">m</span>
                        </div>
                    ))}
                </div>

                <label className="input-label">Velocity Dispersion 1-std [m/s]</label>
                <div className="input-row">
                    {['x', 'y', 'z'].map(k => (
                        <div key={k} className="input-field">
                            <span className="field-label">{k}</span>
                            <input type="number" value={velErr[k]} step={0.001} min={0}
                                onChange={e => setVelErr({ ...velErr, [k]: parseFloat(e.target.value) || 0 })} />
                            <span className="field-unit">m/s</span>
                        </div>
                    ))}
                </div>

                <button className="run-btn" onClick={() => run()} disabled={loading}>
                    {loading ? 'Running MC...' : 'Run Monte Carlo'}
                </button>
            </div>

            {error && <div className="error-msg">{error}</div>}

            {result && (
                <div className="results-section">
                    <table className="results-table">
                        <thead>
                            <tr>
                                <th>Success Rate</th>
                                <th>Samples</th>
                                <th>Mean ΔV</th>
                                <th>Std ΔV</th>
                                <th>Mean Duration</th>
                                <th>Std Duration</th>
                                <th>Mean Range</th>
                            </tr>
                        </thead>
                        <tbody>
                            <tr>
                                <td className={result.successRate > 0.9 ? 'val-ok' : result.successRate > 0.5 ? 'val-warn' : 'val-err'}>
                                    {(result.successRate * 100).toFixed(1)}%
                                </td>
                                <td>{result.nSuccess} / {result.nSamples}</td>
                                <td>{result.meanDV.toFixed(4)} m/s</td>
                                <td>{result.stdDV.toFixed(4)} m/s</td>
                                <td>{result.meanDuration.toFixed(1)} s</td>
                                <td>{result.stdDuration.toFixed(1)} s</td>
                                <td>{result.meanFinalRange.toFixed(2)} m</td>
                            </tr>
                        </tbody>
                    </table>

                    {result.failures && (
                        <table className="results-table" style={{ marginTop: '8px' }}>
                            <thead>
                                <tr>
                                    <th>Position Timeout</th>
                                    <th>Excess Velocity</th>
                                    <th>Limit Cycle</th>
                                    <th>P95 ΔV</th>
                                    <th>P99 ΔV</th>
                                    <th>P95 Duration</th>
                                </tr>
                            </thead>
                            <tbody>
                                <tr>
                                    <td>{result.failures.positionTimeout}</td>
                                    <td>{result.failures.excessVelocity}</td>
                                    <td>{result.failures.limitCycle || 0}</td>
                                    <td>{result.dvPercentiles?.p95?.toFixed(4) || '—'} m/s</td>
                                    <td>{result.dvPercentiles?.p99?.toFixed(4) || '—'} m/s</td>
                                    <td>{result.durationPercentiles?.p95?.toFixed(1) || '—'} s</td>
                                </tr>
                            </tbody>
                        </table>
                    )}

                    <div className="plot-container">
                        <Plot
                            data={scatterTraces}
                            layout={layout3D({
                                title: { text: 'Final Position Dispersion (LVLH)' },
                                ...ranges3D,
                            })}
                            config={PLOTLY_CONFIG}
                            style={{ width: '100%', height: '500px' }}
                        />
                    </div>

                    <div className="plot-row">
                        <div className="plot-half">
                            <Plot
                                data={[{
                                    x: result.samples?.map(s => s.totalDV) || [],
                                    type: 'histogram',
                                    marker: { color: COLORS.trajectory },
                                    opacity: 0.8,
                                }]}
                                layout={layout2D({
                                    title: { text: 'ΔV Distribution' },
                                    xaxis: { title: 'Total ΔV [m/s]' },
                                    yaxis: { title: 'Count' },
                                    showlegend: false,
                                })}
                                config={PLOTLY_CONFIG}
                                style={{ width: '100%', height: '300px' }}
                            />
                        </div>
                        <div className="plot-half">
                            <Plot
                                data={[{
                                    x: result.samples?.map(s => s.duration) || [],
                                    type: 'histogram',
                                    marker: { color: COLORS.amber },
                                    opacity: 0.8,
                                }]}
                                layout={layout2D({
                                    title: { text: 'Duration Distribution' },
                                    xaxis: { title: 'Duration [s]' },
                                    yaxis: { title: 'Count' },
                                    showlegend: false,
                                })}
                                config={PLOTLY_CONFIG}
                                style={{ width: '100%', height: '300px' }}
                            />
                        </div>
                    </div>
                </div>
            )}
        </div>
    );
}

export default MonteCarloPanel;