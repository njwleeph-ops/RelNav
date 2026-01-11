import React from 'react';

const TABS = [
    { id: 'propagator', label: 'CW Propagator' },
    { id: 'montecarlo', label: 'Monte Carlo' },
    { id: 'twoimpulse', label: 'Two-Impulse' },
    { id: 'lqr', label: 'LQR Control' }
];

function TabNav({ activeTab, onTabChange }) {
    return (
        <nav className="tab-nav">
            {TABS.map(tab => (
                <button
                    key={tab.id}
                    className={`tab-btn ${activeTab === tab.id ? 'active' : ''}`}
                    onClick={() => onTabChange(tab.id)}
                >
                    {tab.label}
                </button>
            ))}
        </nav>
    );
}

export default TabNav;