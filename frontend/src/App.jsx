import React, { useState, useEffect, useCallback } from 'react';
import { getHealth, getOrbit } from './api';
import {
    Header,
    TabNav,
    GuidancePanel,
    MonteCarloPanel,
    EnvelopePanel,
} from './Components';
import './App.css';

function App() {
    const [activeTab, setActiveTab] = useState('propagator');
    const [orbitInfo, setOrbitInfo] = useState(null);
    const [backendOk, setBackendOk] = useState(false);

    const [guidanceResult, setGuidanceResult] = useState(null);
    const [mcResult, setMcResult] = useState(null);
    const [envelopeResult, setEnvelopeResult] = useState(null);

    const [mcOverrides, setMcOverrides] = useState(null);

    useEffect(() => {
        getHealth()
            .then(() => setBackendOk(true))
            .catch(() => setBackendOk(false));

        getOrbit(420e3)
            .then(setOrbitInfo)
            .catch(() => { });
    }, []);

    const handleDrillInto = useCallback((params) => {
        setMcOverrides(params);
        setActiveTab('montecarlo');
    }, []);

    const handleMcOverridesConsumed = useCallback(() => {
        setMcOverrides(null);
    }, []);

    return (
        <div className="app">
            <Header orbitInfo={orbitInfo} backendOk={backendOk} />
            <TabNav activetab={activeTab} onTabChange={setActiveTab} />
            <main className="main-content">
                <div style={{ display: activeTab === 'guidance' ? 'block' : 'none' }}>
                    <GuidancePanel
                        savedResult={guidanceResult}
                        onResult={setGuidanceResult}
                    />
                </div>
                <div style={{ display: activeTab === 'montecarlo' ? 'block' :'none' }}>
                    <MonteCarloPanel
                        savedResult={mcResult}
                        onResult={setMcResult}
                        overrides={mcOverrides}
                        onOverridesConsumed={handleMcOverridesConsumed}
                    />
                </div>
                <div style={{ display: activeTab === 'envelope' ? 'block' : 'none' }}>
                    <EnvelopePanel
                        savedResult={envelopeResult}
                        onResult={setEnvelopeResult}
                        onDrillInto={handleDrillInto}
                    />
                </div>
            </main>
            <footer className="footer">
                <p>RelNav-MC v2.1</p>
            </footer>
        </div>
    );
}

export default App;