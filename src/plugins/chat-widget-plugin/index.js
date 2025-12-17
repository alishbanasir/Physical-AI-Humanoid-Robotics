/**
 * Docusaurus plugin to inject ChatWidget into all pages.
 * Registers the widget as a global component that appears on every page.
 * @returns {import('@docusaurus/types').Plugin}
 */
// src/plugins/chat-widget-plugin/index.js
import { resolve } from 'path';

function chatWidgetPlugin() {
    return {
        name: 'chat-widget-plugin',
        getClientModules() {
            // This ensures the clientModule.tsx is correctly located, even on Windows
            return [
                resolve(__dirname, './clientModule.tsx'),
            ];
        },
    };
}

export default chatWidgetPlugin; // Correct CommonJS export for Node.js