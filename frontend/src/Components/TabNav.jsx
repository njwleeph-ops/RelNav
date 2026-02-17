import React from 'react';

const TABS = [
    { id: 'guidance', label: 'Approach Guidance' },
    { id: 'validation', label: 'Validation' },
    { id: 'montecarlo', label: 'Monte Carlo' },
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