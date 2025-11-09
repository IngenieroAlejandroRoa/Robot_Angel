/*
 * Re-export our widget, contribution and DI module.  Theia
 * tooling will compile this file into the `lib` folder.  When
 * referenced via the `frontend` entry in `theiaExtensions` this
 * module will be executed, which will register our bindings.
 */

export * from './angel-widget';
export * from './angel-contribution';
export { default as default } from './angel-frontend-module';