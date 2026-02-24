import React from 'react';

function StateInput({ state, onChange, lockedAxis }) {
    const handleChange = (key, value) => {
        const v = parseFloat(value) || 0;
        
        if (key === lockedAxis && v > 0) {
            onChange({ ...state, [key]: -Math.abs(v) });
        } else {
            onChange({ ...state, [key]: v });
        }
    };

    return (
        <div className="state-input">
            <label className="input-label">Initial State (LVLH)</label>
            <div className="input-row">
                {['x', 'y', 'z'].map(k => (
                    <div key={k} className="input-field">
                        <span className="field-label">
                            {k} {k === lockedAxis ? '(≤ 0)' : ''}
                        </span>
                        <input
                            type="number"
                            value={state[k]}
                            onChange={e => handleChange(k, e.target.value)}
                        />
                        <span className="field-unit">m</span>
                    </div>
                ))}
            </div>
            <div className="input-row">
                {['vx', 'vy', 'vz'].map(k => (
                    <div key={k} className="input-field">
                        <span className="field-label">{k}</span>
                        <input
                            type="number"
                            value={state[k]}
                            onChange={e => handleChange(k, e.target.value)}
                        />
                        <span className="field-unit">m/s</span>
                    </div>    
                ))}
            </div>
        </div>
    );
}

export default StateInput;