/**
 * @file EnvelopePanel.jsx
 * @brief 2D heatmap parameter sweep in accordance with MC analysis for each pair
 */

import React, { useState, useRef, useEffect } from 'react';
import Plot from 'react-plotly.js';
import { startEnvelope as apiStartEnvelope, getEnvelopeStatus } from '../api';
import { darkLayout, PLOTLY_CONFIG, COLORS } from '../constants';

const AXIS_OPTIONS = {
    vbar: { label: 'V-bar', axis: { x: 0,  y: -1, z: 0  } },
    rbar: { label: 'R-bar', axis: { x: -1, y: 0,  z: 0  } },
    hbar: { label: 'H-bar', axis: { x: 0,  y: 0,  z: -1 } },
};

const METRIC_MODES = [
    { id: 'successRate', label: 'Success Rate' },
    { id: 'p95dv', label: 'P95 dv' },
    { id: 'failure', label: 'Failure Mode' },
];

const FAILURE_COLORS = {
    none: '#00ff88',
    position_timeout: '#ff4444',
    excess_velocity: '#ffaa00',
};

const FAILURE_LABELS = {
    none: 'Success',
    position_timeout: 'Position Timeout',
    excess_velocity: 'Excess Velocity',
};

function EnvelopePanel({ onDrillInto }) {
    // --- Sweep Configs ---
    const [approachAxis, setApproachAxis] = useState('vbar');
    const [rangeMin, setRangeMin] = useState(100);
    const [rangeMax, setRangeMax] = useState(3000);
    const [uMaxMin, setUMaxMin] = useState(0.001);
    const [uMaxMax, setUMaxMax] = useState(0.05);
    const [rangeSteps, setRangeSteps] = useState(25);
    const [uMaxSteps, setUMaxSteps] = useState(20);
    const [samplesPerPoint, setSamplesPerPoint] = useState(200);
    const [seed, setSeed] = useState(42);

    // --- Controller tuning ---
    const [qPos, setQPos] = useState(10);
    const [qVel, setQVel] = useState(50);
    const [R, setR] = useState(1);
    const [uMaxNominal, setUMaxNominal] = useState(0.01);
    const [dt, setDt] = useState(1.0);
    const [timeout, setTimeout_] = useState(6000.0);
    const [successRange, setSuccessRange] = useState(5.0);
    const [successVelocity, setSuccessVelocity] = useState(0.05);

    // --- Corridor ---
    const [corridorAngle, setCorridorAngle] = useState(0.175);
    const [glideslopeK, setGlideslopeK] = useState(0.001);
    const [minRange, setMinRange] = useState(10.0);

    // --- Uncertainty ---
    const [posErr, setPosErr] = useState(5.0);
    const [velErr, setVelErr] = useState(0.01);
    const [thrustMagErr, setThrustMagErr] = useState(0.03);
    const [thrustPtErr, setThrustPtErr] = useState(0.0175);

    // --- Job state ---
    const [jobId, setJobId] = useState(null);
    const [progress, setProgress] = useState(0);
    const [status, setStatus] = useState(null);
    const [result, setResult] = useState(null);
    const [error, setError] = useState(null);
    const [computeTime, setComputeTime] = useState(null);
    const pollRef = useRef(null);

    // --- Display ---
    const [metricMode, setMetricMode] = useState('successRate');

    // Cleanup polling on unmount
    useEffect(() => {
        return () => {
            if (pollRef.current) clearInterval(pollRef.current);
        };
    }, []);

    // --- API Calls ---
    const runEnvelope = async() => {
        setError(null);
        setResult(null);
        setProgress(null);
        setComputeTime(null);

        const axisVec = AXIS_OPTIONS[approachAxis].axis;

        const body = {
            altitude: 420e3,
            corridor: {
                axis: axisVec,
                corridorAngle,
                glideslopeK,
                minRange,
            },
            Q: { pos: qPos, vel: qVel },
            R: R,
            uMax: uMaxNominal,
            dt,
            timeout,
            successRange,
            successVelocity,
            uncertainty: {
                posError: { x: posErr, y: posErr, z: posErr },
                velError: { x: velErr, y: velErr, z: velErr },
                thrustMagError: thrustMagErr,
                thrustPointingError: thrustPtErr,
            },
            sweep: {
                rangeMin, rangeMax, rangeSteps,
                uMaxMin, uMaxMax, uMaxSteps,
                samplesPerPoint, seed,
            },
        };

        try {
            const data = await apiStartEnvelope(body);

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
                setResult(data);
                setComputeTime(data.computeTime);
            } else if (data.status === 'failed') {
                clearInterval(pollRef.current);
                pollRef.current = null;
                setError(data.error || 'Envelope computation failed');
            }
        } catch (e) {
            clearInterval(pollRef.current);
            pollRef.current = null;
            setError(e.message);
        }
    };

    // --- Build heatmap data ---
    const buildHeatmap = () => {
        if (!result || !result.grid) return null;

        const nRange = result.rangeValues.length;
        const nUmax = result.uMaxValues.length;

        const z = Array.from({ length: nUmax }, () => Array(nRange).fill(0));
        const text = Array.from({ length: nUmax }, () => Array(nRange).fill(''));

        for (const cell of result.grid) {
            const i = cell.rangeIdx;
            const j = cell.uMaxIdx;

            if (metricMode === 'successRate') {
                z[j][i] = cell.successRate * 100;
                text[j][i] = `${cell.range.toFixed(0)}m, ${cell.uMax.toFixed(4)} m/s^2<br>`
                    + `Success: ${(cell.successRate * 100).toFixed(1)}%<br>`
                    + `${cell.nSuccess}/${cell.nSamples}`;
            } else if (metricMode === 'p95dv') {
                z[j][i] = cell.p95DV;
                text[j][i] = `${cell.range.toFixed(0)}m, ${cell.uMax.toFixed(4)} m/s²<br>`
                    + `P95 ΔV: ${cell.p95DV.toFixed(3)} m/s<br>`
                    + `Mean ΔV: ${cell.meanDV.toFixed(3)} m/s`;
            } else if (metricMode === 'failure') {
                const modeMap = { 'none': 0, 'position_timeout': 1, 'excess_velocity': 2 };
                z[j][i] = modeMap[cell.dominantFailure] ?? 0;
                text[j][i] = `${cell.range.toFixed(0)}m, ${cell.uMax.toFixed(4)} m/s²<br>`
                    + `Dominant: ${FAILURE_LABELS[cell.dominantFailure] || 'Unknown'}<br>`
                    + `Position Timeout: ${cell.failures.positionTimeout} | Excess Velocity: ${cell.failures.excessVelocity}`;
            }
        }

        return { z, text };
    };

    const heatmapData = result ? buildHeatmap() : null;

    // Colorscale per metric
    const getColorscale = () => {
        if (metricMode === 'successRate') {
            return [
                [0, '#ff4444'],
                [0.5, '#ffaa00'],
                [0.8, '#ffdd44'],
                [0.95, '#88ff88'],
                [1, '#00ff88'],
            ];
        } else if (metricMode === 'p95dv') {
            return [
                [0, '#00ff88'],
                [0.3, '#00d4ff'],
                [0.6, '#ffaa00'],
                [1, '#ff4444'],
            ];
        } else {
            return [
                [0, '#00ff88'],
                [0.5, '#ff4444'],
                [1, '#ffaa00'],
            ];
        }
    };

    // --- Click handler: drill into MC tab ---
    const handleCellClick = (event) => {
        if (!result || !event.points || event.points.length === 0) return;

        const pt = event.points[0];

        if (!onDrillInto) return;

        onDrillInto({
            range: pt.x,
            uMax: pt.y,
            approachAxis,
        });
    };

    // --- Render ---
    return (
        <div className="panel">
            <div classname="panel-header">
                <h2>Performance Envelope</h2>
                <p>2D Sweep: Initial range along axis vs thrust authority</p>
            </div>

            <div className="input-selection">
                <h4>Sweep Configuration</h4>
                <div className="input-row">
                    <label>
                        Approach Axis:
                        <select value={approachAxis} onChange={e => setApproachAxis(e.target.value)}>
                            {Object.entries(AXIS_OPTIONS).map(([k, v]) => (
                                <option key={k} value={k}>{v.label}</option>
                            ))}
                        </select>
                    </label>
                </div>
                <div className="input-row">
                    <label>
                        Range Minimum:
                        <input type="number" value={rangeMin} step="50"
                            onChange={e => setRangeMin(parseFloat(e.target.value) || 100)}
                        />
                        m
                    </label>
                    <label>
                        Range Maximum:
                        <input type="numnber" value={rangeMax} step="50"
                            onChange={e => setRangeMax(parseFloat(e.target.value) || 100)}
                        /> m
                    </label>
                    <label>
                        Steps:
                        <input type="number" value={rangeSteps} min="3" max="50"
                            onChange={e => setRangeSteps(parseInt(e.target.value) || 15)}
                        />
                    </label>
                </div>
                <div className="input-row">
                    <label>
                        u_max Minimum:
                        <input type="number" value={uMaxMin} step="0.001"
                            onChange={e => setUMaxMin(parseFloat(e.target.value) || 0.001)}    
                        />
                        m/s^2
                    </label>
                    <label>
                        u_max Maximum:
                        <input type="number" value={uMaxMax} step="0.01"
                            onChange={e => setUMaxMax(parseFloat(e.target.value) || 0.05)}
                        />
                        m/s^2
                    </label>
                    <label>
                        Steps:
                        <input type="number" value={uMaxSteps} min="3" max="50"
                            onChange={e => setUMaxSteps(parseInt(e.target.value) || 12)}
                        />
                    </label>
                </div>
                <div className="input-row">
                    <label>
                        Samples/cell:
                        <input type="number" value={samplesPerPoint} step="50" min="50"
                            onChange={e => setSamplesPerPoint(parseFloat(e.target.value) || 200)}
                        />
                    </label>
                    <label>
                        Seed:
                        <input type="number" value={seed}
                            onChange={e => setSeed(parseInt(e.target.value) || 42)}
                        />
                    </label>                    
                </div>

                <h4>Controller Tuning</h4>
                <div className="input-row">
                    <label>
                        Q_pos:
                        <input type="numnber" value={qPos} step="0.5"
                            onChange={e => setQPos(parseFloat(e.target.value) || 10.0)}
                        />
                    </label>
                    <label>
                        Q_vel:
                        <input type="number" value={qVel} step="0.5"
                            onChange={e => setQVel(parseFloat(e.target.value) || 50.0)}
                        />
                    </label>
                    <label>
                        R:
                        <input type="number" value={R} step="0.5"
                            onChange={e => setR(parseFloat(e.target.value) || 1.0)}
                        />
                    </label>
                </div>
                <div className="input-row">
                    <label>
                        Success Range:
                        <input type="number" value={successRange} min="0.01"
                            onChange={e => setSuccessRange(parseFloat(e.target.value) || 5.0)}
                        />
                        m
                    </label>
                    <label>
                        Success Velocity:
                        <input type="number" value={successVelocity} min="0.001"
                            onChange={e => setSuccessVelocity(parseFloat(e.target.value) || 0.05)}
                        />
                        m/s
                    </label>
                </div>

                <h4>Corridor Parameters</h4>
                <div className="input-row">
                    <label>
                        Corridor Angle:
                        <input type="number" value={corridorAngle} step="0.001"
                            onChange={e => setCorridorAngle(parseFloat(e.target.value) || 0.175)}
                        />
                        rad
                    </label>
                    <label>
                        Glideslope k:
                        <input type="number" value={glideslopeK} step="0.001"
                            onChange={e => setGlideslopeK(parseFloat(e.target.value) || 0.001)}
                        />
                    </label>
                    <label>
                        Minimum Range:
                        <input type="number" value={minRange} step="0.01"
                            onChange={e => setMinRange(parseFloat(e.target.value) || 10.0)}
                        />
                    </label>
                </div>

                <h4>Uncertainty</h4>
                <div className="input-row">
                    <label>
                        Position:
                        <input type="number" value={posErr} step="0.5"
                            onChange={e => setPosErr(parseFloat(e.target.value) || 5)}
                        />
                        m    
                    </label>
                    <label>
                        Velocity:
                        <input type="number" value={velErr} step="0.5"
                            onChange={e => setVelErr(parseFloat(e.target.value) || 0.05)}
                        />
                        m/s
                    </label>
                    <label>
                        Thrust Magnitude:
                        <input type="number" value={thrustMagErr} step="0.01"
                            onChange={e => setThrustMagErr(parseFloat(e.target.value) || 0.03)}
                        />
                        m/s^2
                    </label>
                    <label>
                        Thrust Pointing:
                        <input type="number" value={thrustPtErr} step="0.005"
                            onChange={e => setThrustPtErr(parseFloat(e.target.value) || 0.0175)}
                        />
                        rad
                    </label>
                </div>
            </div>

            <button className="run-btn" onClick={runEnvelope} disabled={status === 'running'}>
                {status === 'running' ? 'Computing...' : 'Compute Envelope'}
            </button>

            {/* Progress Bar */}
            {status === 'running' && (
                <div className="progress-container">
                    <div className="progress-bar">
                        <div className="progress-fill"
                            style={{ width: `${(progress * 100).toFixed(0)}%` }} />
                    </div>
                    <span className="progress-label">
                        {(progress * 100).toFixed(0)}% - {Math.round(progress * rangeSteps * uMaxSteps)}/{rangeSteps * uMaxSteps} cells
                    </span>
                </div>    
            )}

            {error && <div className="error-msg">{error}</div>}

            {/* Results */}
            {result && (
                <>
                    {/* Compute Time */}
                    {computeTime && (
                        <div className="compute-time">
                            Computed in {computeTime.toFixed(1)}s - {result.approachAxis.toUpperCase()} approach
                        </div>
                    )}

                    {/* Metric Toggle */}
                    <div className="metric-toggle">
                        {METRIC_MODES.map(m => (
                            <button key={m.id}
                                className={`toggle-btn ${metricMode === m.id ? 'active': ''}`}
                                onClick={() => setMetricMode(m.id)}
                            >
                                {m.label}
                            </button>
                        ))}
                    </div>

                    {/* Heatmap */}
                    {heatmapData && (
                        <div className="plot-container">
                            <Plot
                                data={[{
                                    z: heatmapData.z,
                                    x: result.rangeValues,
                                    y: result.uMaxValues,
                                    type: 'heatmap',
                                    colorscale: getColorscale(),
                                    text: heatmapData.text,
                                    hoverinfo: 'text',
                                    showscale: metricMode !== 'failure',
                                    colorbar: metricMode === 'successRate'
                                        ? { title: 'Success %', ticksuffix: '%' }
                                        : metricMode === 'p95dv'
                                            ? { title: 'P95 dv [m/s]' }
                                            : {},
                                    zmin: metricMode === 'successRate' ? 0 : undefined,
                                    zmax: metricMode === 'successRate' ? 100
                                        : metricMode === 'failure' ? 3 : undefined,
                                }]}
                                layout={darkLayout({
                                    title: {
                                        text: metricMode === 'successRate' ? 'Success Rate Envelope'
                                            : metricMode === 'p95dv' ? 'P95 dv Envelope'
                                                : 'Dominant Failure Mode',
                                        font: { color: '#c8c8d8' },
                                    },
                                    xaxis: { title: 'Initial Range [m]', type: 'linear' },
                                    yaxis: { title: 'Max Thrust [m/s]', type: 'log' },
                                })}
                                config={PLOTLY_CONFIG}
                                style={{ width: '100%', height: '550px' }}
                                onClick={handleCellClick}
                            />

                            {/* Failure mode legend */}
                            {metricMode === 'failure' && (
                                <div className="failure-legend">
                                    {Object.entries(FAILURE_LABELS).map(([key, label]) => (
                                        <span key={key} className="legend-item">
                                            <span className="legend-swatch"
                                                style={{ backgroundColor: FAILURE_COLORS[key] }} 
                                            />
                                                {label} 
                                        </span>
                                    ))}
                                </div>
                            )}
                            
                            <p className="heatmap-hint"> Click a cell to run MC analysis for parameter set</p>
                        </div>
                    )}
                </>
            )}
        </div>
    );
}

export default EnvelopePanel;