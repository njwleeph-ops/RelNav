import React, { useState, useEffect } from 'react';
import { getHealth, getOrbit } from './api';
import {
    Header,
    TabNav,
    GuidancePanel,
    ValidationPanel,
    MonteCarloPanel
} from './Components';
import './App.css';

function App() {
    const [activeTab, setActiveTab] = useState('propagator');
    const [orbitInfo, setOrbitInfo] = useState(null);
    const [backendOk, setBackendOk] = useState(false);

    useEffect(() => {
        getHealth()
            .then(() => setBackendOk(true))
            .catch(() => setBackendOk(false));

        getOrbit(420e3)
            .then(setOrbitInfo)
            .catch(() => { });
    }, []);

    const renderPanel = () => {
        switch (activeTab) {
            case 'guidance': return <GuidancePanel />;
            case 'validation': return <ValidationPanel />;
            case 'montecarlo': return <MonteCarloPanel />;
            default: return <GuidancePanel />;
        }
    };

    return (
        <div className="app">
            <Header orbitInfo={orbitInfo} backendOk={backendOk}/>
            <TabNav activeTab={activeTab} onTabChange={setActiveTab} />
            <main className="main-content">
                {renderPanel()}
            </main>
            <footer className="footer">
                <p>RelNav-MC v2.0 | C++ Backend + React Frontend</p>
            </footer>
        </div>
    );
}

export default App;