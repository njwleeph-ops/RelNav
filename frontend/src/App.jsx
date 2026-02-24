import React, { useState, useEffect, useCallback } from 'react';
import { getHealth, getOrbitInfo, getConfig } from './api';
import {
    Header,
    TabNav,
    ConfigDrawer,
    GuidancePanel,
    MonteCarloPanel,
    EnvelopePanel,
} from './Components';
import './App.css';

function App() {
    const [activeTab, setActiveTab] = useState('guidance');
    const [backendOk, setBackendOk] = useState(false);
    const [orbitInfo, setOrbitInfo] = useState(null);
    const [config, setConfig] = useState(null);
    const [configOpen, setConfigOpen] = useState(false);

    // Persisted results per panel
    const [guidanceResult, setGuidanceResult] = useState(null);
    const [mcResult, setMcResult] = useState(null);
    const [envelopeResult, setEnvelopeResult] = useState(null);

    // Drill-into: envelope → MC
    const [mcOverrides, setMcOverrides] = useState(null);

    const loadConfig = useCallback(async () => {
        try {
            const cfg = await getConfig();
            setConfig(cfg);
            if (cfg?.orbit?.altitude) {
                const orbit = await getOrbitInfo(cfg.orbit.altitude);
                setOrbitInfo(orbit);
            }
        } catch (e) {
            console.error('Failed to load config:', e);
        }
    }, []);

    useEffect(() => {
        getHealth()
            .then(() => setBackendOk(true))
            .catch(() => setBackendOk(false));
        loadConfig();
    }, [loadConfig]);

    const handleDrillInto = useCallback((params) => {
        setMcOverrides(params);
        setActiveTab('montecarlo');
    }, []);

    const handleMcOverridesConsumed = useCallback(() => {
        setMcOverrides(null);
    }, []);

    return (
        <div className="app">
            <Header
                backendOk={backendOk}
                orbitInfo={orbitInfo}
                onToggleConfig={() => setConfigOpen(!configOpen)}
            />

            <TabNav activeTab={activeTab} onTabChange={setActiveTab} />

            <ConfigDrawer
                config={config}
                open={configOpen}
                onClose={() => setConfigOpen(false)}
                onReload={loadConfig}
            />

            <main className="main-content">
                <div style={{ display: activeTab === 'guidance' ? 'block' : 'none' }}>
                    <GuidancePanel
                        config={config}
                        savedResult={guidanceResult}
                        onResult={setGuidanceResult}
                    />
                </div>
                <div style={{ display: activeTab === 'montecarlo' ? 'block' : 'none' }}>
                    <MonteCarloPanel
                        config={config}
                        savedResult={mcResult}
                        onResult={setMcResult}
                        overrides={mcOverrides}
                        onOverridesConsumed={handleMcOverridesConsumed}
                    />
                </div>
                <div style={{ display: activeTab === 'envelope' ? 'block' : 'none' }}>
                    <EnvelopePanel
                        config={config}
                        savedResult={envelopeResult}
                        onResult={setEnvelopeResult}
                        onDrillInto={handleDrillInto}
                    />
                </div>
            </main>

            <footer className="footer">
                <span>RelNav-MC v3.0</span>
            </footer>
        </div>
    );
}

export default App;