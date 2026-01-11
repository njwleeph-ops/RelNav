import React from 'react';

function Header({ orbitInfo }) {
    return (
        <header className="header">
            <h1>RelNav-MC</h1>
            <p className="subtitle">Spacecraft Proximity Operations Simulator</p>
            {orbitInfo && (
                <div className="orbit-badge">
                    ISS Orbit: {(orbitInfo.altitude / 1000).toFixed(0)} km |
                    Period: {(orbitInfo.period / 60).toFixed(1)} min |
                    n = {(orbitInfo.meanMotion * 1000).toFixed(4)} mrad/s
                </div>
            )}
        </header>
    );
}

export default Header;