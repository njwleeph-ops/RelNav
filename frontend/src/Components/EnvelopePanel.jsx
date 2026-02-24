import React, { useState, useRef, useEffect } from 'react';
import Plot from 'react-plotly.js';
import { startEnvelope, getEnvelopeStatus } from '../api';
import { COLORS, PLOTLY_CONFIG, layout2D, getAxisLabel } from '../constants';

const METRIC_MODES = [
    { id: 'successRate', label: 'Success Rate' },
    { id: 'p95dv', label: 'P95 ΔV' },
    { id: 'failure', label: 'Failure Mode' },
];

const FAILURE_LABELS = {
    none: 'None',
    position_timeout: 'Position Timeout',
    excess_velocity: 'Excess Velocity',
    limit_cycle: 'Limit Cycle',
};

const FAILURE_COLORS_MAP = {
    none: '#16a34a',
    position_timeout: '#dc2626',
    excess_velocity: '#d97706',
    limit_cycle: '#7c3aed',
};

function EnvelopePanel({ config, savedResult, onResult, onDrillInto }) {
    // Sweep config
    const [rangeMin, setRangeMin] = useState(100);
    const [rangeMax, setRangeMax] = useState(3000);
    const [rangeSteps, setRangeSteps] = useState(25);
    const [uMaxMin, setUMaxMin] = useState(0.001);
    const [uMaxMax, setUMaxMax] = useState(0.05);
    const [uMaxSteps, setUMaxSteps] = useState(20);
    const [samplesPerPoint, setSamplesPerPoint] = useState(200);

    // Dispersion
    const [posErr, setPosErr] = useState({ x: 10, y: 10, z: 10 });
    const [velErr, setVelErr] = useState({ x: 0.01, y: 0.01, z: 0.01 });

    // Job state
    const [jobId, setJobId] = useState(null);
    const [progress, setProgress] = useState(0);
    const [status, setStatus] = useState(null);
    const [error, setError] = useState(null);
    const [computeTime, setComputeTime] = useState(null);
    const pollRef = useRef(null);

    // Display
    const [metricMode, setMetricMode] = useState('successRate');

    const result = savedResult;

    useEffect(() => {
        return () => { if (pollRef.current) clearInterval(pollRef.current); };
    }, []);

    const runEnvelope = async () => {
        setError(null);
        setProgress(0);
        setComputeTime(null);

        try {
            const data = await startEnvelope(posErr, velErr, {
                rangeMin, rangeMax, rangeSteps,
                uMaxMin, uMaxMax, uMaxSteps,
                samplesPerPoint,
            });
            setJobId(data.jobId);
            setStatus('running');
            pollRef.current = setInterval(() => pollJob(data.jobId), 2000);
        } catch (e) {
            setError(e.message);
        }
    };

    const pollJob = async (id) => {
        try {
            const data = await getEnvelopeStatus(id);
            setProgress(data.progress || 0);
            setStatus(data.status);

            if (data.status === 'complete') {
                clearInterval(pollRef.current);
                pollRef.current = null;
                onResult(data);
                setComputeTime(data.computeTime);
            } else if (data.status === 'failed') {
                clearInterval(pollRef.current);
                pollRef.current = null;
                setError(data.error || 'Computation failed');
            }
        } catch (e) {
            clearInterval(pollRef.current);
            pollRef.current = null;
            setError(e.message);
        }
    };

    // Build heatmap
    const buildHeatmap = () => {
        if (!result?.grid) return null;

        const nRange = result.rangeValues.length;
        const nUmax = result.uMaxValues.length;
        const z = Array.from({ length: nUmax }, () => Array(nRange).fill(0));
        const text = Array.from({ length: nUmax }, () => Array(nRange).fill(''));

        for (const cell of result.grid) {
            const i = cell.rangeIdx;
            const j = cell.uMaxIdx;

            if (metricMode === 'successRate') {
                z[j][i] = cell.successRate * 100;
                text[j][i] = `Range: ${cell.range.toFixed(0)} m\n`
                    + `u_max: ${cell.uMax.toFixed(4)} m/s²\n`
                    + `Success: ${(cell.successRate * 100).toFixed(1)}% (${cell.nSuccess}/${cell.nSamples})`;
            } else if (metricMode === 'p95dv') {
                z[j][i] = cell.p95DV;
                text[j][i] = `Range: ${cell.range.toFixed(0)} m\n`
                    + `u_max: ${cell.uMax.toFixed(4)} m/s²\n`
                    + `P95 ΔV: ${cell.p95DV.toFixed(4)} m/s\n`
                    + `Mean ΔV: ${cell.meanDV.toFixed(4)} m/s`;
            } else if (metricMode === 'failure') {
                const modeMap = { none: 0, position_timeout: 1, excess_velocity: 2, limit_cycle: 3 };
                z[j][i] = modeMap[cell.dominantFailure] ?? 0;
                text[j][i] = `Range: ${cell.range.toFixed(0)} m\n`
                    + `u_max: ${cell.uMax.toFixed(4)} m/s²\n`
                    + `Dominant: ${FAILURE_LABELS[cell.dominantFailure] || 'Unknown'}\n`
                    + `Timeout: ${cell.failures.positionTimeout} | Velocity: ${cell.failures.excessVelocity}`;
            }
        }

        return { z, text };
    };

    const heatmapData = result ? buildHeatmap() : null;

    const getColorscale = () => {
        if (metricMode === 'successRate') {
            return [[0, '#dc2626'], [0.5, '#d97706'], [0.85, '#fbbf24'], [1, '#16a34a']];
        } else if (metricMode === 'p95dv') {
            return [[0, '#16a34a'], [0.3, '#1a56db'], [0.6, '#d97706'], [1, '#dc2626']];
        } else {
            return [[0, '#16a34a'], [0.33, '#dc2626'], [0.66, '#d97706'], [1, '#7c3aed']];
        }
    };

    const handleCellClick = (event) => {
        if (!result || !event.points || event.points.length === 0) return;
        const pt = event.points[0];
        if (onDrillInto) {
            onDrillInto({ range: pt.x, uMax: pt.y });
        }
    };

    const axisLabel = config ? getAxisLabel(config.glideslope?.approach_axis) : '';

    return (
        <div className="panel">
            <div className="panel-header">
                <h2>Performance Envelope</h2>
            </div>

            <div className="input-section">
                <label className="input-label">Sweep Configuration</label>
                <div className="input-row">
                    <div className="input-field">
                        <span className="field-label">Range Min</span>
                        <input type="number" value={rangeMin} step={50}
                            onChange={e => setRangeMin(parseFloat(e.target.value) || 100)} />
                        <span className="field-unit">m</span>
                    </div>
                    <div className="input-field">
                        <span className="field-label">Range Max</span>
                        <input type="number" value={rangeMax} step={50}
                            onChange={e => setRangeMax(parseFloat(e.target.value) || 3000)} />
                        <span className="field-unit">m</span>
                    </div>
                    <div className="input-field">
                        <span className="field-label">Range Steps</span>
                        <input type="number" value={rangeSteps} min={3} max={50}
                            onChange={e => setRangeSteps(parseInt(e.target.value) || 20)} />
                    </div>
                </div>
                <div className="input-row">
                    <div className="input-field">
                        <span className="field-label">u_max Min</span>
                        <input type="number" value={uMaxMin} step={0.001}
                            onChange={e => setUMaxMin(parseFloat(e.target.value) || 0.001)} />
                        <span className="field-unit">m/s²</span>
                    </div>
                    <div className="input-field">
                        <span className="field-label">u_max Max</span>
                        <input type="number" value={uMaxMax} step={0.01}
                            onChange={e => setUMaxMax(parseFloat(e.target.value) || 0.05)} />
                        <span className="field-unit">m/s²</span>
                    </div>
                    <div className="input-field">
                        <span className="field-label">u_max Steps</span>
                        <input type="number" value={uMaxSteps} min={3} max={50}
                            onChange={e => setUMaxSteps(parseInt(e.target.value) || 15)} />
                    </div>
                </div>
                <div className="input-row">
                    <div className="input-field">
                        <span className="field-label">Samples/Cell</span>
                        <input type="number" value={samplesPerPoint} step={50} min={50}
                            onChange={e => setSamplesPerPoint(parseInt(e.target.value) || 200)} />
                    </div>
                </div>

                <label className="input-label">Position Dispersion 1σ [m]</label>
                <div className="input-row">
                    {['x', 'y', 'z'].map(k => (
                        <div key={k} className="input-field">
                            <span className="field-label">{k}</span>
                            <input type="number" value={posErr[k]} step={1} min={0}
                                onChange={e => setPosErr({ ...posErr, [k]: parseFloat(e.target.value) || 0 })} />
                        </div>
                    ))}
                </div>

                <label className="input-label">Velocity Dispersion 1σ [m/s]</label>
                <div className="input-row">
                    {['x', 'y', 'z'].map(k => (
                        <div key={k} className="input-field">
                            <span className="field-label">{k}</span>
                            <input type="number" value={velErr[k]} step={0.001} min={0}
                                onChange={e => setVelErr({ ...velErr, [k]: parseFloat(e.target.value) || 0 })} />
                        </div>
                    ))}
                </div>

                <button className="run-btn" onClick={runEnvelope} disabled={status === 'running'}>
                    {status === 'running' ? 'Computing...' : 'Compute Envelope'}
                </button>
            </div>

            {status === 'running' && (
                <div className="progress-container">
                    <div className="progress-bar">
                        <div className="progress-fill" style={{ width: `${(progress * 100).toFixed(0)}%` }} />
                    </div>
                    <span className="progress-label">
                        {(progress * 100).toFixed(0)}% — {Math.round(progress * rangeSteps * uMaxSteps)}/{rangeSteps * uMaxSteps} cells
                    </span>
                </div>
            )}

            {error && <div className="error-msg">{error}</div>}

            {result && (
                <div className="results-section">
                    {computeTime && (
                        <div className="compute-time">
                            Computed in {computeTime.toFixed(1)}s — {axisLabel} approach
                        </div>
                    )}

                    <div className="metric-toggle">
                        {METRIC_MODES.map(m => (
                            <button key={m.id}
                                className={`toggle-btn ${metricMode === m.id ? 'active' : ''}`}
                                onClick={() => setMetricMode(m.id)}
                            >
                                {m.label}
                            </button>
                        ))}
                    </div>

                    {heatmapData && (
                        <div className="plot-container">
                            <Plot
                                data={[{
                                    z: heatmapData.z,
                                    x: result.rangeValues,
                                    y: result.uMaxValues,
                                    type: 'heatmap',
                                    zsmooth: 'best',
                                    colorscale: getColorscale(),
                                    text: heatmapData.text,
                                    hoverinfo: 'text',
                                    showscale: metricMode !== 'failure',
                                    colorbar: metricMode === 'successRate'
                                        ? { title: 'Success %', ticksuffix: '%' }
                                        : metricMode === 'p95dv'
                                            ? { title: 'P95 ΔV [m/s]' }
                                            : {},
                                    zmin: metricMode === 'successRate' ? 0 : undefined,
                                    zmax: metricMode === 'successRate' ? 100
                                        : metricMode === 'failure' ? 3 : undefined,
                                }]}
                                layout={layout2D({
                                    title: {
                                        text: metricMode === 'successRate' ? 'Success Rate Envelope'
                                            : metricMode === 'p95dv' ? 'P95 ΔV Envelope'
                                                : 'Dominant Failure Mode',
                                    },
                                    xaxis: { title: 'Initial Range [m]' },
                                    yaxis: { title: 'Max Thrust [m/s²]', type: 'log' },
                                })}
                                config={PLOTLY_CONFIG}
                                style={{ width: '100%', height: '550px' }}
                                onClick={handleCellClick}
                            />

                            {metricMode === 'failure' && (
                                <div className="failure-legend">
                                    {Object.entries(FAILURE_LABELS).map(([key, label]) => (
                                        <span key={key} className="legend-item">
                                            <span className="legend-swatch"
                                                style={{ backgroundColor: FAILURE_COLORS_MAP[key] }} />
                                            {label}
                                        </span>
                                    ))}
                                </div>
                            )}

                            <p className="heatmap-hint">Click a cell to drill into Monte Carlo analysis</p>
                        </div>
                    )}
                </div>
            )}
        </div>
    );
}

export default EnvelopePanel;