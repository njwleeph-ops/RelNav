import React from 'react';

function Header({ orbitInfo, backendOk }) {
    return (
        <header className="header">
            <div className="header-left">
                <h1>RelNav-MC</h1>
                <p className="subtitle">Rendezvous Proximity Operations Simulation</p>
            </div>
            <div className="header-right">
                <span className={`status-dot ${backendOk ? 'ok': 'err'}`}/>
                <span className="status-label">
                    {backendOk ? 'Backend connected' : 'Backend offline'}
                </span>
                {orbitInfo && (
                    <span className="orbit-badge">
                        ISS {(orbitInfo.altitude / 1000).toFixed(0)} km
                        &nbsp;|&nbsp;T = {orbitInfo.periodMinutes.toFixed(1)} min
                        &nbsp;|&nbsp;n = {(orbitInfo.meanMotion * 1000).toFixed(4)} mrad/s
                    </span>
                )}
            </div>
        </header>
    );
}

export default Header;