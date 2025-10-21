// @ts-check

require('reflect-metadata');
require('setimmediate');
const { Container } = require('inversify');
const { FrontendApplicationConfigProvider } = require('@theia/core/lib/browser/frontend-application-config-provider');

FrontendApplicationConfigProvider.set({
    "applicationName": "Eclipse Theia",
    "defaultTheme": {
        "light": "light",
        "dark": "dark"
    },
    "defaultIconTheme": "theia-file-icons",
    "electron": {
        "windowOptions": {},
        "showWindowEarly": true
    },
    "defaultLocale": "",
    "validatePreferencesSchema": true
});


self.MonacoEnvironment = {
    getWorkerUrl: function (moduleId, label) {
        return './editor.worker.js';
    }
}

function load(container, jsModule) {
    return Promise.resolve(jsModule)
        .then(containerModule => container.load(containerModule.default));
}

async function preload(parent) {
    const container = new Container();
    container.parent = parent;
    try {
        await load(container, import('@theia/core/lib/browser/preload/preload-module'));
        const { Preloader } = require('@theia/core/lib/browser/preload/preloader');
        const preloader = container.get(Preloader);
        await preloader.initialize();
    } catch (reason) {
        console.error('Failed to run preload scripts.');
        if (reason) {
            console.error(reason);
        }
    }
}

module.exports = (async () => {
    const { messagingFrontendModule } = require('@theia/core/lib/electron-browser/messaging/electron-messaging-frontend-module');
    const container = new Container();
    container.load(messagingFrontendModule);
    await preload(container);
    const { FrontendApplication } = require('@theia/core/lib/browser');
    const { frontendApplicationModule } = require('@theia/core/lib/browser/frontend-application-module');    
    const { loggerFrontendModule } = require('@theia/core/lib/browser/logger-frontend-module');

    container.load(frontendApplicationModule);
    container.load(loggerFrontendModule);

    try {
        await load(container, import('@theia/core/lib/browser/i18n/i18n-frontend-module'));
        await load(container, import('@theia/core/lib/electron-browser/menu/electron-menu-module'));
        await load(container, import('@theia/core/lib/electron-browser/window/electron-window-module'));
        await load(container, import('@theia/core/lib/electron-browser/keyboard/electron-keyboard-module'));
        await load(container, import('@theia/core/lib/electron-browser/token/electron-token-frontend-module'));
        await load(container, import('@theia/core/lib/electron-browser/request/electron-browser-request-module'));
        await load(container, import('@theia/variable-resolver/lib/browser/variable-resolver-frontend-module'));
        await load(container, import('@theia/editor/lib/browser/editor-frontend-module'));
        await load(container, import('@theia/filesystem/lib/browser/filesystem-frontend-module'));
        await load(container, import('@theia/filesystem/lib/browser/download/file-download-frontend-module'));
        await load(container, import('@theia/filesystem/lib/browser/file-dialog/file-dialog-module'));
        await load(container, import('@theia/filesystem/lib/electron-browser/file-dialog/electron-file-dialog-module'));
        await load(container, import('@theia/messages/lib/browser/messages-frontend-module'));
        await load(container, import('@theia/workspace/lib/browser/workspace-frontend-module'));
        await load(container, import('@theia/markers/lib/browser/problem/problem-frontend-module'));
        await load(container, import('@theia/outline-view/lib/browser/outline-view-frontend-module'));
        await load(container, import('@theia/monaco/lib/browser/monaco-frontend-module'));
        await load(container, import('@theia/navigator/lib/browser/navigator-frontend-module'));
        await load(container, import('@theia/navigator/lib/electron-browser/electron-navigator-module'));
        await load(container, import('@theia/userstorage/lib/browser/user-storage-frontend-module'));
        await load(container, import('@theia/preferences/lib/browser/preference-frontend-module'));
        await load(container, import('@theia/process/lib/common/process-common-module'));
        await load(container, import('@theia/terminal/lib/browser/terminal-frontend-module'));
        await load(container, import('@theia/callhierarchy/lib/browser/callhierarchy-frontend-module'));
        await load(container, import('@theia/bulk-edit/lib/browser/bulk-edit-frontend-module'));
        await load(container, import('@theia/console/lib/browser/console-frontend-module'));
        await load(container, import('@theia/output/lib/browser/output-frontend-module'));
        await load(container, import('@theia/task/lib/browser/task-frontend-module'));
        await load(container, import('@theia/debug/lib/browser/debug-frontend-module'));
        await load(container, import('@theia/editor-preview/lib/browser/editor-preview-frontend-module'));
        await load(container, import('@theia/file-search/lib/browser/file-search-frontend-module'));
        await load(container, import('@theia/notebook/lib/browser/notebook-frontend-module'));
        await load(container, import('@theia/scm/lib/browser/scm-frontend-module'));
        await load(container, import('@theia/search-in-workspace/lib/browser/search-in-workspace-frontend-module'));
        await load(container, import('@theia/test/lib/browser/view/test-view-frontend-module'));
        await load(container, import('@theia/timeline/lib/browser/timeline-frontend-module'));
        await load(container, import('@theia/typehierarchy/lib/browser/typehierarchy-frontend-module'));
        await load(container, import('@theia/plugin-ext/lib/plugin-ext-frontend-module'));
        await load(container, import('@theia/plugin-ext/lib/plugin-ext-frontend-electron-module'));
        await load(container, import('@theia/plugin-ext-vscode/lib/browser/plugin-vscode-frontend-module'));
        await load(container, import('@theia/vsx-registry/lib/common/vsx-registry-common-module'));
        await load(container, import('@theia/vsx-registry/lib/browser/vsx-registry-frontend-module'));
        await start();
    } catch (reason) {
        console.error('Failed to start the frontend application.');
        if (reason) {
            console.error(reason);
        }
    }

    function start() {
        (window['theia'] = window['theia'] || {}).container = container;
        return container.get(FrontendApplication).start();
    }
})();
