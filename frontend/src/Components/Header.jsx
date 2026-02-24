import React from 'react';

function Header({ backendOk, orbitInfo, onToggleConfig }) {
    return (
        <header className="header">
            <div className="header-left">
                <h1>RelNav-MC</h1>
                <span className="header-subtitle">Proximity Operations Simulator</span>
            </div>
            <div className="header-right">
                {orbitInfo && (
                    <span className="header-orbit">
                        {(orbitInfo.altitude / 1000).toFixed(0)} km | T = {orbitInfo.periodMinutes.toFixed(1)} min
                    </span>
                )}
                <span className={`header-status ${backendOk ? 'ok' : 'err'}`}>
                    {backendOk ? 'Connected' : 'Disconnected'}
                </span>
                <button className="config-btn" onClick={onToggleConfig} title="View Configuration">
                    Config
                </button>
            </div>
        </header>
    );
}

export default Header;