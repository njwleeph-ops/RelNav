import React, { useState } from 'react';
import { reloadConfig } from '../api';

function ConfigDrawer({ config, open, onClose, onReload }) {
    const [reloading, setReloading] = useState(false);
    const [msg, setMsg] = useState(null);

    const handleReload = async () => {
        setReloading(true);
        setMsg(null);
        try {
            await reloadConfig();
            if (onReload) await onReload();
            setMsg('Config reloaded');
        } catch (e) {
            setMsg('Reload failed: ' + e.message);
        } finally {
            setReloading(false);
        }
    };

    if (!open) return null;

    const flattenObject = (obj, prefix = '') => {
        const rows = [];
        for (const [key, val] of Object.entries(obj)) {
            const fullKey = prefix ? `${prefix}.${key}` : key;
            if (val !== null && typeof val === 'object' && !Array.isArray(val)) {
                rows.push(...flattenObject(val, fullKey));
            } else if (Array.isArray(val)) {
                rows.push({ key: fullKey, val: `[${val.join(', ')}]` });
            } else {
                rows.push({ key: fullKey, val: String(val) });
            }
        }
        return rows;
    };

    const renderSection = (title, data) => {
        if (!data) return null;
        const rows = flattenObject(data);
        return (
            <div className="config-section">
                <h4>{title}</h4>
                <table className="config-table">
                    <tbody>
                        {rows.map(({ key, val }) => (
                            <tr key={key}>
                                <td className="config-key">{key}</td>
                                <td className="config-val">{val}</td>
                            </tr>
                        ))}
                    </tbody>
                </table>
            </div>
        );
    };

    return (
        <div className="config-overlay" onClick={onClose}>
            <div className="config-drawer" onClick={e => e.stopPropagation()}>
                <div className="config-drawer-header">
                    <h3>GNC Configuration</h3>
                    <button className="config-close" onClick={onClose}>&times;</button>
                </div>

                <div className="config-drawer-actions">
                    <button
                        className="reload-btn"
                        onClick={handleReload}
                        disabled={reloading}
                    >
                        {reloading ? 'Reloading...' : 'Reload from YAML'}
                    </button>
                    {msg && <span className="config-msg">{msg}</span>}
                </div>

                <div className="config-drawer-body">
                    {config ? (
                        <>
                            {renderSection('Orbit', config.orbit)}
                            {renderSection('LQR Controller', config.lqr)}
                            {renderSection('Glideslope', config.glideslope)}
                            {renderSection('Nav Filter', config.nav_filter)}
                            {renderSection('Control', config.control)}
                            {renderSection('Simulation', config.sim)}
                            {renderSection('Monte Carlo', config.mc)}
                        </>
                    ) : (
                        <p>No config loaded</p>
                    )}
                </div>

                <div className="config-drawer-footer">
                    <span>Edit gnc_config.yaml and reload</span>
                </div>
            </div>
        </div>
    );
}

export default ConfigDrawer;